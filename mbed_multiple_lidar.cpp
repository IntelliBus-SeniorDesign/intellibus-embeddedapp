#include "mbed.h"
#include "VL53L0X.h"
Serial pc(USBTX, USBRX); 
 
DevI2C      i2c(p9, p10);
DigitalOut  shdn1(p11), shdn2(p12);
VL53L0X     ld1(&i2c, &shdn1, NC), ld2(&i2c, &shdn2, NC); 
 
int main() {
    // Turn off all VL53L0X sensors (by grounding the shutdown pin)
    ld1.VL53L0X_off();
    ld2.VL53L0X_off();
    
    // Program the new I2C addresses
    ld1.init_sensor(0x30);
    ld2.init_sensor(0x50);
    uint32_t distance =0;
    uint32_t distance2 = 0; 
    int status;
    int status2; 
    
    while(1) {
        status = ld1.get_distance(&distance); 
        status2 = ld2.get_distance(&distance2); 
        if(status == VL53L0X_ERROR_NONE)
        {   
            pc.printf("Distance 1: %ld mm\r\n", distance); 
        }
        //wait(0.5); 
        if(status2 == VL53L0X_ERROR_NONE)
        {
            pc.printf("Distance 2: %ld mm\r\n", distance2); 
        }
        //wait(0.5); 
        
    }
}

