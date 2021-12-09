#include "Adafruit_VL53L0X.h"
/******************************************************************************
* Author : Thomas Talbot 
* Last Date Modified : 11/23/2021
* Purpose : Using the Adafruit VL53L0X library for the Arduino Nano, if the 
* sensor detects a mm reading less than the predefined threshold then output
* a logic HIGH signal to the corresponding digital out pin. 
******************************************************************************/
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31


// The pins for shutdown
//Sensor 1 - A2
//Sensor 2 - A3

//The Digital Out pins if a sensor has reading < threshold
//Sensor 1 - A6
//Sensor 2 - A7

//The threshold value to detect motion
#define threshold 200 

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(A2, LOW);    
  digitalWrite(A3, LOW);
  delay(10);
  // all unreset
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(A3, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
    if(measure1.RangeMilliMeter<threshold)
    {
      Serial.println(F(" Sensor 1 triggered!")); 
      digitalWrite(6, HIGH); 
    }
    else{
      digitalWrite(6, LOW); 
    }
  } else {
    Serial.print(F("Out of range"));
    //Must set pin 6 low if out of range for counting logic to work properly
    digitalWrite(6, LOW); 
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
    if(measure2.RangeMilliMeter< threshold)
    {
      Serial.println(F(" Sensor 2 triggered!")); 
      digitalWrite(7, HIGH);
      //sensor2responses++; 
    }
    else
    {
      digitalWrite(7, LOW); 
    }
  } else {
    Serial.print(F("Out of range"));
    digitalWrite(7, LOW); 
  }
  
  Serial.println();
 
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(6, OUTPUT); 
  pinMode(7, OUTPUT); 


  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);


  digitalWrite(6, LOW);
  digitalWrite(7, LOW); 

  setID();
 
}

void loop() {
   
  read_dual_sensors();
  delay(100);
}
