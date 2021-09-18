********************************************************************
* Author: Thomas W. Talbot
* Last Date Modified: 09/17/21
* Continually Blink LED2 on the Nordic Board 
*********************************************************************/
#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <drivers/gpio.h>

/* 1000 mse = 1 sec */
#define SLEEP_TIME 100
#define LED 3

void main(void)
{
	int cnt = 0;
	struct device *dev;

	dev = device_get_binding("GPIO_0");
	/* Set LED pin as output */
	gpio_pin_configure(dev, LED, GPIO_DIR_MASK); //p0.03 == LED2

	while (1) {
		/* Set pin to HIGH/LOW every 0.1 second */
		gpio_pin_set(dev, LED, 0);
                k_sleep(K_MSEC(SLEEP_TIME)); 
                gpio_pin_set(dev, LED, 1); 
                k_sleep(K_MSEC(SLEEP_TIME));

	}
}
