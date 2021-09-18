/********************************************************************
* Author: Thomas W. Talbot
* Last Date Modified: 09/18/21
* Continually Read GPIO p0_24 and p0_25 to check PIR sensor 1 and 2
* If a passenger crosses sensor 1 before sensor 2 they are leaving.
*********************************************************************/
#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include <sys/mutex.h>
#include <sys/atomic.h>

/* 1000 mse = 1 sec */
#define SLEEP_TIME 100
#define LED_1 2
#define LED_2 3
#define pin_s1 24 //Assume Sensor 1 is on inside of bus. 
#define pin_s2 25

bool sense1High = false; 
bool sense2High = false; 
bool entering = false; 
int psg_count =0; 

void main(void)
{
        printf("Starting program!\n");
	struct device *dev;

	dev = device_get_binding("GPIO_0");
	/* Set LED pins as output */
	gpio_pin_configure(dev, LED_1, GPIO_DIR_MASK); //p0.02 == LED1
        gpio_pin_configure(dev, LED_2, GPIO_DIR_MASK); //p0.03 == LED2

        /* Set the PIR sensor pins as inputs*/
        gpio_pin_configure(dev, pin_s1, GPIO_INPUT); 
	gpio_pin_configure(dev, pin_s2, GPIO_INPUT); 
        unsigned s1_val = 0U; 
        unsigned s2_val = 0U; 
	while (1) 
        {
            unsigned prev_s1_val = s1_val; 
            unsigned prev_s2_val = s2_val; 
            s1_val = gpio_pin_get(dev, pin_s1); 
            s2_val = gpio_pin_get(dev, pin_s2); 

            if(s1_val && !prev_s1_val) 
            { 
               //Rising edge of s1
               printf("Sensor 1 triggered!\n");
               gpio_pin_set(dev, LED_1, 1); 
               
               sense1High = true; 

            }
            if(s2_val && !prev_s2_val)
            {
                //Rising edge of s2
                printf("Sensor 2 triggered!\n"); 
                gpio_pin_set(dev, LED_2, 1); 
                sense2High = true; 
            }
            if(!s1_val && prev_s1_val) 
            {
                //Falling edge of s1
                //Leave boolean high 
                gpio_pin_set(dev, LED_1, 0); 
            }
            if(!s2_val && prev_s2_val) 
            {
                //Falling edge of s2
                //Leave booolean high 
                gpio_pin_set(dev, LED_2, 0); 
            }
            
            if(sense1High && !sense2High)
            {
                //Sensor 1 is closest to the inside of the bus 
                entering = false; 
            }
            else if(!sense1High && sense2High)
            {
                //Sensor 2 triggered before sensor 1
                entering = true; 
            }
            else if(sense1High && sense2High)
            {
                //The passenger has crossed both sensors
                if(entering)
                {
                    psg_count++; 
                }
                else if(psg_count >0) //no negative passenger counts 
                {
                    psg_count--; 
                }
                //reset the sensors for the next person 
                sense1High = false; 
                sense2High = false; 
                k_sleep(K_MSEC(SLEEP_TIME));  
                printf("The number of passengers is: %d\n", psg_count); 
             

            }

	}
}

