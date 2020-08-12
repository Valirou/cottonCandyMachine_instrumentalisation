/*---------------------------------------------------------------------------------------------------
 * File   : _main.ino
 * Author : Valérie Toupin-Guay
 * Date   : 2020-08-12
 *  
 *  Main program of the instrumentalization's circuit for a cotton candy machine workin 
 *  with 120 VAC. It reads the current of the heating circuit. It also reads the number 
 *  of revolutions of the rotative head. Finaly, it reads the head temperature with an 
 *  infrared sensor. With the temperature reading, it maintains the head temperature to 
 *  a setpoint using a PID control. All data, are shown in the serial monitor.
 *  
 *  The data will be used for researches. The control of the temperature is needed
 *  to guarantee good fibers every time excluding external factor. 
 *  
 *  
 *  This fonction does not have any entries or returns
 *   
 *   Calling:                       
 *   obtainData();
---------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------
 * Components's connections
 *  
 * **************************************************************************************************
 *  Infrared temperature sensor
 *  from https://abra-electronics.com/sensors/sensors-temperature-en/sens-57-no-contact-digital-ir-thermometer-sensor.html
 *  Address (I2C) : 
 *  
 *  MLX90614  → Arduino MEGA or intermediate bus
 *  VIN       → 5V bus
 *  GND       → GND bus
 *  SCL       → SCL 21 bus
 *  SDA       → SDA 20 bus
 *  **************************************************************************************************
 *  Current sensor
 *  from https://abra-electronics.com/sensors/sensors-current-en/sen0098-50a-current-sensorac-dc-sen0098.html
 *  Address (I2C) :
 *  
 *  SEN0098 (ACS758)  → Arduino MEGA or intermediate bus
 *  VCC               → 5V bus
 *  GND               → GND bus
 *  VIOUT             → A0
 *  In AC circuit :
 *  IN                → SSR ...
 *  OUT               → heating element
 *  **************************************************************************************************
 *  Proximity sensor
 *  from https://abra-electronics.com/sensors/sensors-proximity-en/466-vcnl4000-proximity-light-sensor.html
 *  Address (I2C) :
 *  
 *  VCNL4010  → Arduino MEGA or intermediate bus
 *  VIN       → 5V bus
 *  GND       → GND bus
 *  SCL       → SCL 21 bus
 *  SDA       → SDA 20 bus
 *  **************************************************************************************************
 *  Solid state relay (SSR)
 *  from ...
 *  Address (I2C) : none
 *  
 *  ...               → Arduino MEGA or intermediate bus
 *  ...               → PWM 2
 *  GND               → GND bus
 *  In AC circuit :
 *  ...               → heating switch (120 VAC supply)
 *  ...               → heating element
 *  **************************************************************************************************
 ---------------------------------------------------------------------------------------------------*/

// Outside librairies used in this code ( see https://www.arduino.cc/reference/en/libraries/ )
#include <Adafruit_MLX90614.h> // to read the temperature from the sensor
#include <Wire.h> // to communicate with I2C / TWI devices ( MEGA → pins : 20 (SDA), 21 (SCL) ) 


// Constants and variables's definition
// MEGA's inputs
const int VIOUT_CURRENT_SENSOR_PIN = A0;
// MEGA's outputs
const int SSR_PULSE_PIN = 2;
// PID constants
// PID variables
// Temperature constant
// Temperature variables
// Current constants
const 
// Current variables
// Revolution constants
// Revolution variables


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

/*---------------------------------------------------------------------------------------------------
 * Code references:
 * [1] AC 220V Heater Temperature PID and TRIAC control. (2018, avril 8). https://www.youtube.com/watch?v=P6mbBJDIvxI
 * [2] Build An Infrared Thermometer. (2020, mars 28). https://www.youtube.com/watch?v=UID87M-IKsg
 * [3] Measure current with ACS758 Current Sensor and LCD1602-I2C with Arduino. (2018, juin 20). https://www.youtube.com/watch?v=tug9wjCwDQA
 * [4] VCNL4000 Entfernungssensor, Helligkeitssensor am Arduino. (2014, décembre 16). https://www.youtube.com/watch?v=Rp5NcXffsBE 
---------------------------------------------------------------------------------------------------*/
