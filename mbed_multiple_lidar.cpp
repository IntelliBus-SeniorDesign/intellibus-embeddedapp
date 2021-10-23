#include "mbed.h"
#include "VL53L0X.h"
Serial pc(USBTX, USBRX); 
DigitalOut led1(LED1); 
DigitalOut led2(LED2); 
 
DevI2C      i2c(p9, p10);
DigitalOut  shdn1(p11), shdn2(p12);
VL53L0X     ld1(&i2c, &shdn1, NC), ld2(&i2c, &shdn2, NC); 
float threshold_mm = 110.00; 
bool sense1High = false; 
bool sense2High = false; 
bool entering = false; 
int psg_count =0; 
 
int main() {
    led1 = 0; 
    led2 =0; 
    // Turn off all VL53L0X sensors (by grounding the shutdown pin)
    ld1.VL53L0X_off();
    ld2.VL53L0X_off();
    
    // Program the new I2C addresses
    ld1.init_sensor(0x30);
    ld2.init_sensor(0x50);
    uint32_t distance1 =0;
    uint32_t distance2 = 0; 
    int status;
    int status2; 
    
    while(1) {
        status = ld1.get_distance(&distance1); 
        status2 = ld2.get_distance(&distance2); 
        while(status != VL53L0X_ERROR_NONE)
        {   
            ld1.get_distance(&distance1); 
        }
        pc.printf("Distance 1: %ld mm\r\n", distance1); 
        wait(0.1); 
        while(status2 != VL53L0X_ERROR_NONE)
        {
            ld2.get_distance(&distance2); 
        }
        pc.printf("Distance 2: %ld mm\r\n", distance2); 
        wait(0.1); 
        
        if(distance1 < threshold_mm)
        {
            pc.printf("Sensor 1 triggered\r\n"); 
            sense1High =true; 
            led1 =1; 
        }
        if(distance2< threshold_mm) 
        {
            pc.printf("Sensor 2 triggered\r\n"); 
            sense2High =true; 
            led2 = 1; 
        }
        if(sense1High && !sense2High)
        {
            entering = true; 
        }
        else if(!sense1High && sense2High)
        {
            entering = false; 
        }
        else if(sense1High && sense2High)
        {
            //The passenger has crossed booth sensors
            if(entering)
            {
                psg_count++; 
            }
            else
            {
                psg_count--; 
            }
            //reset the sensors for the next person 
            sense1High = false; 
            sense2High = false; 
            led1 =0; 
            led2 =0; 
                //k_sleep(K_MSEC(SLEEP_TIME));  
            printf("The number of passengers is: %d\n", psg_count); 
            //wait(5); 
                //break; 

        }//end else if
    }//end while
}//end main 
