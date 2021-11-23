/*
 * Author: Yue (David) Pan
 * 
 * This is a multithreading implementation of executing AWS_IOT project and infrared sensor counting simultaneously.
 * 
 * This program sends the current passenger count to the project's AWS cloud. The passenger count comes from trigerrings of the GPIO pins.
 * 
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>

#if defined(CONFIG_NRF_MODEM_LIB)
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>
#include <modem/modem_info.h>
#include <nrf_modem.h>
#endif

#include <net/aws_iot.h>
#include <sys/reboot.h>
#include <date_time.h>
#include <dfu/mcuboot.h>
#include <cJSON.h>
#include <cJSON_os.h>

// GPIO counting dependencies
#include <device.h>
#include <drivers/gpio.h>
#include <sys/mutex.h>
#include <sys/atomic.h>
//#include <kernel.h>


// GPS dependencies
#include <nrf_modem_gnss.h>
#include <string.h>
#include <modem/at_cmd.h>
#include <modem/lte_lc.h>


// GPS Macros
#ifdef CONFIG_SUPL_CLIENT_LIB
#include <supl_os_client.h>
#include <supl_session.h>
#include "supl_support.h"
#endif

#define AT_XSYSTEMMODE      "AT\%XSYSTEMMODE=0,0,1,0"
#define AT_ACTIVATE_GPS     "AT+CFUN=31"


#define AT_CMD_SIZE(x) (sizeof(x) - 1)

static const char update_indicator[] = {'\\', '|', '/', '-'};
static const char *const at_commands[] = {
#if !defined(CONFIG_SUPL_CLIENT_LIB)
	AT_XSYSTEMMODE,
#endif
	CONFIG_GPS_SAMPLE_AT_MAGPIO,
	CONFIG_GPS_SAMPLE_AT_COEX0,
	AT_ACTIVATE_GPS
};

static struct nrf_modem_gnss_pvt_data_frame last_pvt;   
static volatile bool gnss_blocked;

#ifdef CONFIG_SUPL_CLIENT_LIB
static struct nrf_modem_gnss_agps_data_frame last_agps;
static struct k_work_q agps_work_q;
static struct k_work get_agps_data_work;

#define AGPS_WORKQ_THREAD_STACK_SIZE 2048
#define AGPS_WORKQ_THREAD_PRIORITY   5

#define max_iterations_delay 150

K_THREAD_STACK_DEFINE(agps_workq_stack_area, AGPS_WORKQ_THREAD_STACK_SIZE);
#endif

K_MSGQ_DEFINE(nmea_queue, sizeof(struct nrf_modem_gnss_nmea_data_frame *), 10, 4);
K_SEM_DEFINE(pvt_data_sem, 0, 1);
K_SEM_DEFINE(lte_ready, 0, 1);

struct k_poll_event events[2] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&pvt_data_sem, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&nmea_queue, 0),
};

void nrf_modem_recoverable_error_handler(uint32_t error)
{
	printk("Modem library recoverable error: %u\n", error);
}

static int gps_setup_modem(void)
{
	for (int i = 0; i < ARRAY_SIZE(at_commands); i++) {
        printk("[GPS] Sending AT command `%s`\n", at_commands[i]); 
		if (at_commands[i][0] == '\0') {
			continue;
		}

		if (at_cmd_write(at_commands[i], NULL, 0, NULL) != 0) {
			return -1;
		}
	}

	return 0;
}


#ifdef CONFIG_SUPL_CLIENT_LIB
BUILD_ASSERT(IS_ENABLED(CONFIG_LTE_NETWORK_MODE_LTE_M_GPS) ||
	     IS_ENABLED(CONFIG_LTE_NETWORK_MODE_NBIOT_GPS) ||
	     IS_ENABLED(CONFIG_LTE_NETWORK_MODE_LTE_M_NBIOT_GPS),
	     "To use SUPL and GPS, CONFIG_LTE_NETWORK_MODE_LTE_M_GPS, "
	     "CONFIG_LTE_NETWORK_MODE_NBIOT_GPS or "
	     "CONFIG_LTE_NETWORK_MODE_LTE_M_NBIOT_GPS must be enabled");

static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
		    (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			printk("Connected to LTE network\n");
			k_sem_give(&lte_ready);
		}
		break;
	default:
		break;
	}
}

static int activate_lte(bool activate)
{
	int err;

	if (activate) {
		err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_LTE);
		if (err) {
			printk("Failed to activate LTE, error: %d\n", err);
			return -1;
		}

		printk("LTE activated\n");
		k_sem_take(&lte_ready, K_FOREVER);
	} else {
		err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);
		if (err) {
			printk("Failed to deactivate LTE, error: %d\n", err);
			return -1;
		}

		printk("LTE deactivated\n");
	}

	return 0;
}

static void get_agps_data(struct k_work *item)
{
	ARG_UNUSED(item);

	printk("\033[1;1H");
	printk("\033[2J");
	printk("New A-GPS data requested, contacting SUPL server, flags %d\n",
			last_agps.data_flags);

	activate_lte(true);

	if (open_supl_socket() == 0) {
		printk("Starting SUPL session\n");
		supl_session(&last_agps);
		printk("Done\n");
		close_supl_socket();
	}
	activate_lte(false);
}

static int inject_agps_type(void *agps, size_t agps_size, uint16_t type, void *user_data)
{
	ARG_UNUSED(user_data);

	int retval = nrf_modem_gnss_agps_write(agps, agps_size, type);

	if (retval != 0) {
		printk("Failed to write A-GPS data, type: %d (errno: %d)\n", type, errno);
		return -1;
	}

	printk("Injected A-GPS data, type: %d, size: %d\n", type, agps_size);

	return 0;
}
#endif

static void gnss_event_handler(int event)
{
	int retval;
	struct nrf_modem_gnss_nmea_data_frame *nmea_data;

	switch (event) {
	case NRF_MODEM_GNSS_EVT_PVT:
		retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
		if (retval == 0) {
			k_sem_give(&pvt_data_sem);
		}
		break;

	case NRF_MODEM_GNSS_EVT_NMEA:
		nmea_data = k_malloc(sizeof(struct nrf_modem_gnss_nmea_data_frame));
		if (nmea_data == NULL) {
			printk("Failed to allocate memory for NMEA\n");
			break;
		}

		retval = nrf_modem_gnss_read(nmea_data,
					     sizeof(struct nrf_modem_gnss_nmea_data_frame),
					     NRF_MODEM_GNSS_DATA_NMEA);
		if (retval == 0) {
			retval = k_msgq_put(&nmea_queue, &nmea_data, K_NO_WAIT);
		}

		if (retval != 0) {
			k_free(nmea_data);
		}
		break;

	case NRF_MODEM_GNSS_EVT_AGPS_REQ:
#ifdef CONFIG_SUPL_CLIENT_LIB
		retval = nrf_modem_gnss_read(&last_agps,
					     sizeof(last_agps),
					     NRF_MODEM_GNSS_DATA_AGPS_REQ);
		if (retval == 0) {
			k_work_submit_to_queue(&agps_work_q, &get_agps_data_work);
		}
#endif
		break;

	case NRF_MODEM_GNSS_EVT_BLOCKED:
		gnss_blocked = true;
		break;

	case NRF_MODEM_GNSS_EVT_UNBLOCKED:
		gnss_blocked = false;
		break;

	default:
		break;
	}
}

static int gps_init_app(void)
{
#ifdef CONFIG_SUPL_CLIENT_LIB
	if (lte_lc_init()) {
		printk("Failed to initialize LTE link controller\n");
		return -1;
	}

	lte_lc_register_handler(lte_handler);

	static struct supl_api supl_api = {
		.read       = supl_read,
		.write      = supl_write,
		.handler    = inject_agps_type,
		.logger     = supl_logger,
		.counter_ms = k_uptime_get
	};

	k_work_queue_start(
		&agps_work_q,
		agps_workq_stack_area,
		K_THREAD_STACK_SIZEOF(agps_workq_stack_area),
		AGPS_WORKQ_THREAD_PRIORITY,
		NULL);

	k_work_init(&get_agps_data_work, get_agps_data);

	if (supl_init(&supl_api) != 0) {
		printk("Failed to initialize SUPL library\n");
		return -1;
	}
#endif /* CONFIG_SUPL_CLIENT_LIB */


    // We do not need to setup modem again, since aws_iot project does so already
    
;
	/* Initialize and configure GNSS */
	if (nrf_modem_gnss_init() != 0) {
		printk("Failed to initialize GNSS interface\n");
		return -1;
	}
    
    printk("[GPS] Initialized GNSS interface\n");

	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
		printk("Failed to set GNSS event handler\n");
		return -1;
	}
    
    printk("[GPS] Set GNSS event handler\n");

	if (nrf_modem_gnss_nmea_mask_set(NRF_MODEM_GNSS_NMEA_RMC_MASK |
					 NRF_MODEM_GNSS_NMEA_GGA_MASK |
					 NRF_MODEM_GNSS_NMEA_GLL_MASK |
					 NRF_MODEM_GNSS_NMEA_GSA_MASK |
					 NRF_MODEM_GNSS_NMEA_GSV_MASK) != 0) {
		printk("Failed to set GNSS NMEA mask\n");
		return -1;
	}
    
    printk("[GPS] Set GNSS NMEA mask\n");

	if (nrf_modem_gnss_fix_retry_set(0) != 0) {
		printk("Failed to set GNSS fix retry\n");
		return -1;
	}
    
    printk("[GPS] Set GNSS fix retry\n");

	if (nrf_modem_gnss_fix_interval_set(1) != 0) {
		printk("Failed to set GNSS fix interval\n");
		return -1;
	}
    
    printk("[GPS] Set GNSS fix interval\n");

	if (nrf_modem_gnss_start() != 0) {
		printk("Failed to start GNSS\n");
		return -1;
	}
    
    printk("[GPS] Started GNSS\n");

	return 0;
}

static void print_satellite_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	uint8_t tracked   = 0;
	uint8_t in_fix    = 0;
	uint8_t unhealthy = 0;

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i) {
		if (pvt_data->sv[i].sv > 0) {
			tracked++;

			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
				in_fix++;
			}

			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY) {
				unhealthy++;
			}
		}
	}

	printk("Tracking: %d Using: %d Unhealthy: %d\n", tracked, in_fix, unhealthy);
}

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	printk("Latitude:   %.06f\n", pvt_data->latitude);
	printk("Longitude:  %.06f\n", pvt_data->longitude);
	printk("Altitude:   %.01f m\n", pvt_data->altitude);
	printk("Accuracy:   %.01f m\n", pvt_data->accuracy);
	printk("Speed:      %.01f m/s\n", pvt_data->speed);
	printk("Heading:    %.01f deg\n", pvt_data->heading);
	printk("Date:       %02u-%02u-%02u\n", pvt_data->datetime.year,
					       pvt_data->datetime.month,
					       pvt_data->datetime.day);
	printk("Time (UTC): %02u:%02u:%02u\n", pvt_data->datetime.hour,
					       pvt_data->datetime.minute,
					       pvt_data->datetime.seconds);
}

static bool agps_data_download_ongoing(void)
{
#ifdef CONFIG_SUPL_CLIENT_LIB
	return k_work_is_pending(&get_agps_data_work);
#else
	return false;
#endif
}




// Constants for GPIO counting program
#define LED_1 2
#define LED_2 3
#define pin_s1 24 //Assume Sensor 1 is on inside of bus. 
#define pin_s2 25


BUILD_ASSERT(!IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT),
		"This sample does not support LTE auto-init and connect");

#define APP_TOPICS_COUNT CONFIG_AWS_IOT_APP_SUBSCRIPTION_LIST_COUNT

static struct k_work_delayable shadow_update_work;
static struct k_work_delayable connect_work;
static struct k_work shadow_update_version_work;
static bool cloud_connected;


// Global variable for passenger count
static volatile int passenger_count;

K_SEM_DEFINE(lte_connected, 0, 1);

static int json_add_obj(cJSON *parent, const char *str, cJSON *item)
{
	cJSON_AddItemToObject(parent, str, item);

	return 0;
}

static int json_add_str(cJSON *parent, const char *str, const char *item)
{
	cJSON *json_str;

	json_str = cJSON_CreateString(item);
	if (json_str == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_str);
}

static int json_add_number(cJSON *parent, const char *str, double item)
{
	cJSON *json_num;

	json_num = cJSON_CreateNumber(item);
	if (json_num == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_num);
}

static int shadow_update(bool version_number_include)
{
	int err;
	char *message;
	int64_t message_ts = 0;
	int16_t bat_voltage = 0;

	err = date_time_now(&message_ts);
	if (err) {
		printk("date_time_now, error: %d\n", err);
		return err;
	}

#if defined(CONFIG_NRF_MODEM_LIB)
	/* Request battery voltage data from the modem. */
	err = modem_info_short_get(MODEM_INFO_BATTERY, &bat_voltage);
	if (err != sizeof(bat_voltage)) {
		printk("modem_info_short_get, error: %d\n", err);
		return err;
	}
#endif

	cJSON *root_obj = cJSON_CreateObject();
	cJSON *state_obj = cJSON_CreateObject();
	cJSON *reported_obj = cJSON_CreateObject();

	if (root_obj == NULL || state_obj == NULL || reported_obj == NULL) {
		cJSON_Delete(root_obj);
		cJSON_Delete(state_obj);
		cJSON_Delete(reported_obj);
		err = -ENOMEM;
		return err;
	}

	if (version_number_include) {
		err = json_add_str(reported_obj, "app_version",
				    CONFIG_APP_VERSION);
	} else {
		err = 0;
	}

    /*
    
    Put the passenger count and other information into the `reported_obj`.
    Putting it elsewhere is not ideal. 
    
    */
    
	err += json_add_number(reported_obj, "batv", bat_voltage);
	err += json_add_number(reported_obj, "ts", message_ts);
    err += json_add_number(reported_obj, "passenger count", passenger_count);
    
	err += json_add_obj(state_obj, "reported", reported_obj);
	err += json_add_obj(root_obj, "state", state_obj);

	if (err) {
		printk("json_add, error: %d\n", err);
		goto cleanup;
	}

	message = cJSON_Print(root_obj);
	if (message == NULL) {
		printk("cJSON_Print, error: returned NULL\n");
		err = -ENOMEM;
		goto cleanup;
	}

	struct aws_iot_data tx_data = {
		.qos = MQTT_QOS_0_AT_MOST_ONCE,
		.topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE,
		.ptr = message,
		.len = strlen(message)
	};

	printk("Publishing: %s to AWS IoT broker\n", message);

	err = aws_iot_send(&tx_data);
	if (err) {
		printk("aws_iot_send, error: %d\n", err);
	}

	cJSON_FreeString(message);

cleanup:

	cJSON_Delete(root_obj);

	return err;
}



static void connect_work_fn(struct k_work *work)
{
	int err;

	if (cloud_connected) {
		return;
	}

	err = aws_iot_connect(NULL);
	if (err) {
		printk("aws_iot_connect, error: %d\n", err);
	}

	printk("Next connection retry in %d seconds\n",
	       CONFIG_CONNECTION_RETRY_TIMEOUT_SECONDS);

	k_work_schedule(&connect_work,
			K_SECONDS(CONFIG_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

static void shadow_update_work_fn(struct k_work *work)
{
	int err;

	if (!cloud_connected) {
		return;
	}

	err = shadow_update(false);
	if (err) {
		printk("shadow_update, error: %d\n", err);
	}

	printk("Next data publication in %d seconds\n",
	       CONFIG_PUBLICATION_INTERVAL_SECONDS);

	k_work_schedule(&shadow_update_work,
			K_SECONDS(CONFIG_PUBLICATION_INTERVAL_SECONDS));
}

static void shadow_update_version_work_fn(struct k_work *work)
{
	int err;

	err = shadow_update(true);
	if (err) {
		printk("shadow_update, error: %d\n", err);
	}
}

static void print_received_data(const char *buf, const char *topic,
				size_t topic_len)
{
	char *str = NULL;
	cJSON *root_obj = NULL;

	root_obj = cJSON_Parse(buf);
	if (root_obj == NULL) {
		printk("cJSON Parse failure");
		return;
	}

	str = cJSON_Print(root_obj);
	if (str == NULL) {
		printk("Failed to print JSON object");
		goto clean_exit;
	}

	printf("Data received from AWS IoT console:\nTopic: %.*s\nMessage: %s\n",
	       topic_len, topic, str);

	cJSON_FreeString(str);

clean_exit:
	cJSON_Delete(root_obj);
}

void aws_iot_event_handler(const struct aws_iot_evt *const evt)
{
	switch (evt->type) {
	case AWS_IOT_EVT_CONNECTING:
		printk("AWS_IOT_EVT_CONNECTING\n");
		break;
	case AWS_IOT_EVT_CONNECTED:
		printk("AWS_IOT_EVT_CONNECTED\n");

		cloud_connected = true;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&connect_work);

		if (evt->data.persistent_session) {
			printk("Persistent session enabled\n");
		}

#if defined(CONFIG_NRF_MODEM_LIB)
		/** Successfully connected to AWS IoT broker, mark image as
		 *  working to avoid reverting to the former image upon reboot.
		 */
		boot_write_img_confirmed();
#endif

		/** Send version number to AWS IoT broker to verify that the
		 *  FOTA update worked.
		 */
		k_work_submit(&shadow_update_version_work);

		/** Start sequential shadow data updates.
		 */
		k_work_schedule(&shadow_update_work,
				K_SECONDS(CONFIG_PUBLICATION_INTERVAL_SECONDS));

#if defined(CONFIG_NRF_MODEM_LIB)
		int err = lte_lc_psm_req(true);
		if (err) {
			printk("Requesting PSM failed, error: %d\n", err);
		}
#endif
		break;
	case AWS_IOT_EVT_READY:
		printk("AWS_IOT_EVT_READY\n");
		break;
	case AWS_IOT_EVT_DISCONNECTED:
		printk("AWS_IOT_EVT_DISCONNECTED\n");
		cloud_connected = false;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&shadow_update_work);
		k_work_schedule(&connect_work, K_NO_WAIT);
		break;
	case AWS_IOT_EVT_DATA_RECEIVED:
		printk("AWS_IOT_EVT_DATA_RECEIVED\n");
		print_received_data(evt->data.msg.ptr, evt->data.msg.topic.str,
				    evt->data.msg.topic.len);
		break;
	case AWS_IOT_EVT_FOTA_START:
		printk("AWS_IOT_EVT_FOTA_START\n");
		break;
	case AWS_IOT_EVT_FOTA_ERASE_PENDING:
		printk("AWS_IOT_EVT_FOTA_ERASE_PENDING\n");
		printk("Disconnect LTE link or reboot\n");
#if defined(CONFIG_NRF_MODEM_LIB)
		err = lte_lc_offline();
		if (err) {
			printk("Error disconnecting from LTE\n");
		}
#endif
		break;
	case AWS_IOT_EVT_FOTA_ERASE_DONE:
		printk("AWS_FOTA_EVT_ERASE_DONE\n");
		printk("Reconnecting the LTE link");
#if defined(CONFIG_NRF_MODEM_LIB)
		err = lte_lc_connect();
		if (err) {
			printk("Error connecting to LTE\n");
		}
#endif
		break;
	case AWS_IOT_EVT_FOTA_DONE:
		printk("AWS_IOT_EVT_FOTA_DONE\n");
		printk("FOTA done, rebooting device\n");
		aws_iot_disconnect();
		sys_reboot(0);
		break;
	case AWS_IOT_EVT_FOTA_DL_PROGRESS:
		printk("AWS_IOT_EVT_FOTA_DL_PROGRESS, (%d%%)",
		       evt->data.fota_progress);
	case AWS_IOT_EVT_ERROR:
		printk("AWS_IOT_EVT_ERROR, %d\n", evt->data.err);
		break;
	case AWS_IOT_EVT_FOTA_ERROR:
		printk("AWS_IOT_EVT_FOTA_ERROR");
		break;
	default:
		printk("Unknown AWS IoT event type: %d\n", evt->type);
		break;
	}
}

static void work_init(void)
{
	k_work_init_delayable(&shadow_update_work, shadow_update_work_fn);
	k_work_init_delayable(&connect_work, connect_work_fn);
	k_work_init(&shadow_update_version_work, shadow_update_version_work_fn);
}

#if defined(CONFIG_NRF_MODEM_LIB)
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		     (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			break;
		}

		printk("Network registration status: %s\n",
			evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
			"Connected - home network" : "Connected - roaming");

		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		printk("PSM parameter update: TAU: %d, Active time: %d\n",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE: {
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
			       "eDRX parameter update: eDRX: %f, PTW: %f",
			       evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0) {
			printk("%s\n", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		printk("RRC mode: %s\n",
			evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
			"Connected" : "Idle");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		printk("LTE cell changed: Cell ID: %d, Tracking area: %d\n",
			evt->cell.id, evt->cell.tac);
		break;
	default:
		break;
	}
}

static void modem_configure(void)
{
	int err;

	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
		/* Do nothing, modem is already configured and LTE connected. */
	} else {
        // setup_modem();  // for GPS
        
        printk("[AWS_IOT] Configure modem\n");
		err = lte_lc_init_and_connect_async(lte_handler);
		if (err) {
			printk("Modem could not be configured, error: %d\n",
				err);
			return;
		}
	}
}

static void nrf_modem_lib_dfu_handler(void)
{
	int err;

	err = nrf_modem_lib_get_init_ret();

	switch (err) {
	case MODEM_DFU_RESULT_OK:
		printk("Modem update suceeded, reboot\n");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case MODEM_DFU_RESULT_UUID_ERROR:
	case MODEM_DFU_RESULT_AUTH_ERROR:
		printk("Modem update failed, error: %d\n", err);
		printk("Modem will use old firmware\n");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case MODEM_DFU_RESULT_HARDWARE_ERROR:
	case MODEM_DFU_RESULT_INTERNAL_ERROR:
		printk("Modem update malfunction, error: %d, reboot\n", err);
		sys_reboot(SYS_REBOOT_COLD);
		break;
	default:
		break;
	}
}
#endif

static int app_topics_subscribe(void)
{
	int err;
	static char custom_topic[75] = "my-custom-topic/example";
	static char custom_topic_2[75] = "my-custom-topic/example_2";

	const struct aws_iot_topic_data topics_list[APP_TOPICS_COUNT] = {
		[0].str = custom_topic,
		[0].len = strlen(custom_topic),
		[1].str = custom_topic_2,
		[1].len = strlen(custom_topic_2)
	};

	err = aws_iot_subscription_topics_add(topics_list,
					      ARRAY_SIZE(topics_list));
	if (err) {
		printk("aws_iot_subscription_topics_add, error: %d\n", err);
	}

	return err;
}

static void date_time_event_handler(const struct date_time_evt *evt)
{
	switch (evt->type) {
	case DATE_TIME_OBTAINED_MODEM:
		printk("DATE_TIME_OBTAINED_MODEM\n");
		break;
	case DATE_TIME_OBTAINED_NTP:
		printk("DATE_TIME_OBTAINED_NTP\n");
		break;
	case DATE_TIME_OBTAINED_EXT:
		printk("DATE_TIME_OBTAINED_EXT\n");
		break;
	case DATE_TIME_NOT_OBTAINED:
		printk("DATE_TIME_NOT_OBTAINED\n");
		break;
	default:
		break;
	}
}


void gpio_sensors_count(void) {
    k_sleep(K_MSEC(20000));
    
    //const int SLEEP_TIME = 10000;
    
    printk("start gpio_sensors_count thread\n");
    const struct device *dev;
    dev = device_get_binding("GPIO_0");
    
    gpio_pin_configure(dev, LED_1, GPIO_DIR_MASK); //p0.02 == LED1
    gpio_pin_configure(dev, LED_2, GPIO_DIR_MASK); //p0.03 == LED2

    /* Set the PIR sensor pins as inputs*/
    gpio_pin_configure(dev, pin_s1, GPIO_INPUT); 
	gpio_pin_configure(dev, pin_s2, GPIO_INPUT); 
    
    unsigned s1_val      = 0U; 
    unsigned s2_val      = 0U; 
    unsigned prev_s1_val = 0U;
    unsigned prev_s2_val = 0U;
	unsigned sense1HighCount =0U;  //The  number of iterations where sense1High is true and s1_val is false
	
	//order to handle edge cases, where order of triggers is reversed.
	//If s1_val is and sense1High is true this can only occur for 3 seconds. (150 iterations)


    bool sense1High = false; 
    bool sense2High = false; 
    bool entering   = false; 

    
    while (1) {
        //printk("loop\n");

        prev_s1_val = s1_val;
        prev_s2_val = s2_val;
        
        s1_val = gpio_pin_get(dev, pin_s1); 
        s2_val = gpio_pin_get(dev, pin_s2); 

		if(sense1High && !s1_val)
		{
			sense1HighCount++;
			if(sense1HighCount > max_iterations_delay)
			{
				sense1High = false; //ignore the rising edge
				sense1HighCount =0; //reset the  count 
			}
		}  
		if(sense2High && !s2_val)
		{
			sense2HighCount++; 
			if(sense2HighCount > max_iterations_delay)
			{
				sense2High = false; //ignore the rising edge
				sense1HighCOunt =0; 
			}
		}  
        if (s1_val && !prev_s1_val) 
        { 
           //Rising edge of s1
           printf("Sensor 1 triggered!\n");
           gpio_pin_set(dev, LED_1, 1); 
       
           sense1High = true; 
		   sense1HighCount =0U; 
        }
       
        if (s2_val && !prev_s2_val)
        {
            //Rising edge of s2
            printf("Sensor 2 triggered!\n"); 
            gpio_pin_set(dev, LED_2, 1); 
            sense2High = true; 
        }
        
        if (!s1_val && prev_s1_val) 
        {
            //Falling edge of s1
            //Leave boolean high 
            gpio_pin_set(dev, LED_1, 0); 
        }
        
        if (!s2_val && prev_s2_val) 
        {
            //Falling edge of s2
            //Leave booolean high 
            gpio_pin_set(dev, LED_2, 0); 
        }

        if (sense1High && !sense2High)
        {
            //Sensor 1 is closest to the inside of the bus 
            entering = false; 
        }
        else if (!sense1High && sense2High)
        {
            //Sensor 2 triggered before sensor 1
            entering = true; 
        }
        
        else if (sense1High && sense2High)
        {
            //The passenger has crossed both sensors
            if (entering)
            {
                passenger_count++; 
            }
            else if (passenger_count > 0) //no negative passenger counts 
            {
                passenger_count--; 
            }
            //reset the sensors for the next person 
            sense1High = false; 
			sense1HighCount =0; 
            sense2High = false; 
			sense2HighCount =0; 
        
            printf("The number of passengers is: %d\n", passenger_count); 
        }
        k_sleep(K_MSEC(20));  
    }
}




int gps_main(void)
{
	uint8_t cnt = 0;
	uint64_t fix_timestamp = 0;
	struct nrf_modem_gnss_nmea_data_frame *nmea_data;

	printk("[GPS] Starting GPS thread\n");

	if (gps_init_app() != 0) {
		return -1;
	}

	printk("Getting GNSS data...\n");

	for (;;) {
		(void) k_poll(events, 2, K_FOREVER);

		if (events[0].state == K_POLL_STATE_SEM_AVAILABLE &&
		    k_sem_take(events[0].sem, K_NO_WAIT) == 0) {
			/* New PVT data available */

			if (!IS_ENABLED(CONFIG_GPS_SAMPLE_NMEA_ONLY) &&
			    !agps_data_download_ongoing()) {
				printk("\033[1;1H");
				printk("\033[2J");
				print_satellite_stats(&last_pvt);

				if (gnss_blocked) {
					printk("GNSS operation blocked by LTE\n");
				}
				printk("---------------------------------\n");

				if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
					fix_timestamp = k_uptime_get();
					print_fix_data(&last_pvt);
				} else {
					printk("Seconds since last fix: %lld\n",
							(k_uptime_get() - fix_timestamp) / 1000);
					cnt++;
					printk("Searching [%c]\n", update_indicator[cnt%4]);
				}

				printk("\nNMEA strings:\n\n");
			}
		}
		if (events[1].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE &&
		    k_msgq_get(events[1].msgq, &nmea_data, K_NO_WAIT) == 0) {
			/* New NMEA data available */

			if (!agps_data_download_ongoing()) {
				printk("%s", nmea_data->nmea_str);
			}
			k_free(nmea_data);
		}

		events[0].state = K_POLL_STATE_NOT_READY;
		events[1].state = K_POLL_STATE_NOT_READY;
	}

	return 0;
}


    
static struct k_thread gpio_sensor_thread;
static K_THREAD_STACK_DEFINE(gpio_sensor_thread_stack, 4096);

//static struct k_thread gps_thread;
//static K_THREAD_STACK_DEFINE(gps_thread_stack, 8192);

void main(void)
{
	int err;

	printk("[MAIN] The AWS IoT sample started, version: %s\n", CONFIG_APP_VERSION);
    
	cJSON_Init();
    
  
    
#if defined(CONFIG_NRF_MODEM_LIB)
	nrf_modem_lib_dfu_handler();
#endif

	err = aws_iot_init(NULL, aws_iot_event_handler);
    
    
    
	if (err) {
		printk("AWS IoT library could not be initialized, error: %d\n",
		       err);
	}

	/** Subscribe to customizable non-shadow specific topics
	 *  to AWS IoT backend.
	 */
	err = app_topics_subscribe();
	if (err) {
		printk("Adding application specific topics failed, error: %d\n",
			err);
	}

	work_init();
    
    
    
    k_thread_create(
           &gpio_sensor_thread,                     // thread pointer
           gpio_sensor_thread_stack, 2048,            // stack pointer, stack size
           (k_thread_entry_t) gpio_sensors_count,   // entry point
           NULL, NULL, NULL,                        // param 1, param 2, param 3
           K_PRIO_COOP(7), 0, K_NO_WAIT             // priority, options, delay
    );
    
    
    printk("[MAIN] GPIO thread created\n");
    
#if defined(CONFIG_NRF_MODEM_LIB)


    // Configure modem for GPS and AWS_IOT
	 modem_configure();
    // gps_setup_modem();
    
   
	err = modem_info_init();
	if (err) {
		printk("[MAIN] Failed initializing modem info module, error: %d\n",
			err);
	}

	k_sem_take(&lte_connected, K_FOREVER);
#endif


	date_time_update_async(date_time_event_handler);
	k_work_schedule(&connect_work, K_NO_WAIT);
    
}
