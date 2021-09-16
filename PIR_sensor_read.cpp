#include "mbed.h"
/*******************************************************************************
* Author: Thomas W. Talbot 
* Last Date Modified: 09/16/21
* The purpose of this program is to continually read from PIR sensors to detect 
* the motion of passengers entering and exiting the bus 
*******************************************************************************/
DigitalOut led1(LED1);
DigitalOut led2(LED2); 
InterruptIn sensor1(p30); 
InterruptIn sensor2(p29);

bool sense1High = false; 
bool sense2High = false;  
bool entering = false; 
int psg_count = 0; 

void interrupt1(void)
{
    sense1High = true;  
}

void interrupt2(void)
{
    sense2High = true; 
}

void transmitCount(); //Every so often transmit the passneger count data. 

int main() {
    sensor1.rise(&interrupt1); 
    sensor2.rise(&interrupt2); 
    //Major assumption: Sensor 1 and sensor 2 will trigger
    //Assume that sensors are placed close enough together such that 
    //A new person entering the bus will not trigger sensor1. 
    
    while(1) {
        led1 = sense1High; 
        led2 = sense2High; 
        if(sense1High && !sense2High) 
        {
            //someone is entering 
            entering =true; 
        }
        else if(!sense1High && sense2High)
        {
            //Someone is leaving 
            entering = false; 
        }
        else if(sense1High && sense2High) 
        {
            //Then increment or decrement the count 
            if(entering) 
            {
                psg_count++; 
            }
            else
            {
                psg_count--; 
            }
            //reset the sensors for next person
            sense1High = false; 
            sense2High = false; 
            wait(0.05); 
        
        }
    }
}
