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
 *   _main();
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
 *  SEN0098 (ACS758LCB-050B)  → Arduino MEGA or intermediate bus
 *  VCC               → 5V bus
 *  GND               → GND bus
 *  VIOUT             → A0
 *  In AC circuit :
 *  IN                → SSR ...
 *  OUT               → heating element
 *  **************************************************************************************************
 *  Proximity sensor
 *  from https://abra-electronics.com/sensors/sensors-proximity-en/466-vcnl4000-proximity-light-sensor.html
 *  Address (I2C) : 0x13
 *  
 *  VCNL4010  → Arduino MEGA or intermediate bus
 *  VIN       → 5V bus
 *  GND       → GND bus
 *  SCL       → SCL 21 bus
 *  SDA       → SDA 20 bus
 *  **************************************************************************************************
 *  Solid state relay (SSR)
 *  from http://www.crydom.com/en/products/panel-mount/perfect-fit/ac-output/series-1/d2450/
 *  Address (I2C) : none
 *  
 *  D2450           → Arduino MEGA or intermediate bus
 *  3               → PWM 2
 *  4               → GND bus
 *  In AC circuit :
 *  1               → heating switch (120 VAC supply)
 *  2               → heating element
 *  **************************************************************************************************
 ---------------------------------------------------------------------------------------------------*/

// Outside librairies used in this code ( see https://www.arduino.cc/reference/en/libraries/ )

#include <Adafruit_MLX90614.h> // to read the temperature from the sensor
#include <Wire.h> // to communicate with I2C / TWI devices ( MEGA → pins : 20 (SDA), 21 (SCL) )
#include <PID.h> // to control a PWN pin depending on a reading (proportional–integral–derivative controller)



// Constants and variables's definition

// MEGA's inputs
const int VIOUT_CURRENT_SENSOR_PIN = A0;
const int SDA_PIN = 20;
const int SCL_PIN = 21;

// MEGA's outputs
const int SSR_PULSE_PIN = 2;

// PID constants and object
const double TEMP_SETPOINT = 160; // °C
const double TEMP_RANGE =  0.5; // °C
const double KP = 1; // proportional's coefficient
const double KI = 0; // integral's coefficient 
const double KD = 0; // derivative's coefficient
PID temp_PID = PID( SSR_PULSE_PIN, TEMP_SETPOINT, TEMP_RANGE, KP, KI, KD );

// PID variable
double temp_correction; // ratio

// Temperature constant and object
float TEMP_ERROR = 8; // °C , rectification of the temperature because of the physical montage. Error determined with experimental test and comparison.
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Temperature variables
float ambiantTemp; // °C
float headTemp; // °C

// Current constants (ref [3])
const float VCC = 5.0; // Volts
const int CURRENT_SENSOR_MODEL = 0; // ACS758LCB-050B
/* Reference number used in CURRENT_SENSOR_MODEL in fonction of the sensor's model
          0 → "ACS758LCB-050B"
          1 → "ACS758LCB-050U"
          2 → "ACS758LCB-100B"
          3 → "ACS758LCB-100U"
          4 → "ACS758KCB-150B"
          5 → "ACS758KCB-150U"
          6 → "ACS758ECB-200B"
          7 → "ACS758ECB-200U"            
*/
const float CURRENT_SENSOR_SENSITIVITY[] ={
          40.0,// for ACS758LCB-050B
          60.0,// for ACS758LCB-050U
          20.0,// for ACS758LCB-100B
          40.0,// for ACS758LCB-100U
          13.3,// for ACS758KCB-150B
          16.7,// for ACS758KCB-150U
          10.0,// for ACS758ECB-200B
          20.0,// for ACS758ECB-200U     
         }; // mV/A
const float QUIESCENT_OUTPUT_VOLTAGE[] ={
          0.5,// for ACS758LCB-050B
          0.12,// for ACS758LCB-050U
          0.5,// for ACS758LCB-100B
          0.12,// for ACS758LCB-100U
          0.5,// for ACS758KCB-150B
          0.12,// for ACS758KCB-150U
          0.5,// for ACS758ECB-200B
          0.12,// for ACS758ECB-200U            
          }; // coefficient
const float MY_CURRENT_SENSOR_SENSITIVITY = CURRENT_SENSOR_SENSITIVITY[CURRENT_SENSOR_MODEL]/1000; // mV/A · (1V / 1000 mV) → V/A
const float MY_QOV = QUIESCENT_OUTPUT_VOLTAGE[CURRENT_SENSOR_MODEL] * VCC; // Volts
const float MAKE_ZERO_V = 0.007; // Volts, to make 0V if there is no current see line where this const is use

// Current variables
float cutOffLimit = 1.00; // Amps
float cutOff = MY_CURRENT_SENSOR_SENSITIVITY * cutOffLimit; // V/A · A → Volts
float voltage_read; // Volts
float voltage; // Volts
float current; // Amps

// Revolution constants (ref [4])
const uint8_t VCNL4000_ADDRESS = 0x13;
const uint8_t VCNL4000_COMMAND = 0x80;
const uint8_t VCNL4000_PRODUCTID = 0x81;
const uint8_t VCNL4000_IRLED = 0x83;
const uint8_t VCNL4000_AMBIENTPARAMETER = 0x84;
const uint8_t VCNL4000_AMBIENTDATA = 0x85;
const uint8_t VCNL4000_PROXIMITYDATA = 0x87;
const uint8_t VCNL4000_SIGNALFREQ = 0x89;
const uint8_t VCNL4000_PROXINITYADJUST = 0x8A;
const int VCNL4000_3M125 = 0;
const int VCNL4000_1M5625 = 1;
const int VCNL4000_781K25 = 2;
const int VCNL4000_390K625 = 3;
const uint8_t VCNL4000_MEASUREAMBIENT = 0x10;
const uint8_t VCNL4000_MEASUREPROXIMITY = 0x08;
const uint8_t VCNL4000_AMBIENTREADY = 0x40;
const uint8_t VCNL4000_PROXIMITYREADY = 0x20;
const int MIN_PROXIMITY = 2100; // unknown unit

// Revolution variable
int rpm; // revolution per minute
int revolution; // complete rotation
unsigned int proximity; // unknown unit

// Conversion factor
const float ANALOG_SCALE_TO_V = (5.0 / 1023.0); // coefficient (Ref [5])

// Serial constant
const int SERIAL_SPEED = 9600; // bps → bits per second



// Configuration of the system
void setup() {

   // Open serial port & set his data rate
  Serial.begin(SERIAL_SPEED);
  
  // "Initiate the Wire library and join the I2C bus as a master" (Ref [6])
  Wire.begin();
  
  // Affect the mode of each use pins
  pinMode( VIOUT_CURRENT_SENSOR_PIN, INPUT );
  pinMode( SDA_PIN, INPUT ); // Not sure about this line
  pinMode( SCL_PIN, INPUT ); // Not sure about this line
  pinMode( SSR_PULSE_PIN, OUTPUT );

  // Enable reading from sensors
  mlx.begin(); 
}



// Continuous communications of the system
void loop() {

  // Read the temperature
  headTemp = TEMP_ERROR + mlx.readObjectTempC();
  ambiantTemp = mlx.readAmbientTempC();
  
  // Find the correction to apply in fonction of PWM pins's duty cycle → between 0 & 255 (ref [7])
  temp_correction = temp_PID.run(headTemp);
  
  // Apply correction for the heating element
  analogWrite( SSR_PULSE_PIN, temp_correction );

  // Read the voltage
  voltage_read = ANALOG_SCALE_TO_V * analogRead(VIOUT_CURRENT_SENSOR_PIN);

  // Voltage in fonction of quiescent output voltage
  voltage = voltage_read - MY_QOV + MAKE_ZERO_V;

  // Current in fonction of the voltage(QOV) and the sensitivity
  current = voltage / MY_CURRENT_SENSOR_SENSITIVITY;

  // Read the proximity
  proximity = read16(VCNL4000_PROXIMITYDATA);
/*
  // Count a revolution
  if ( MIN_PROXIMITY < proximity ) {
    revolution += 1;
  }
*/ // brainstorm, to be elaborate
  
  // Display of all data in the serial monitor (table)
  
}



// Fonctions used in setup() and loop()

// Read 16 bits (2 bytes) of data from the proximity sensor (VCNL4000) at a specified addresse (Ref [4])
uint16_t read16(uint8_t address)
{
  uint16_t data;
 
  Wire.beginTransmission(VCNL4000_ADDRESS);
#if ARDUINO >= 100
  Wire.write(address);
#else
  Wire.send(address);
#endif
  Wire.endTransmission();
 
  Wire.requestFrom(VCNL4000_ADDRESS, 2);
  while(!Wire.available());
#if ARDUINO >= 100
  data = Wire.read();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.read();
#else
  data = Wire.receive();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.receive();
#endif
 
  return data;
}


/*---------------------------------------------------------------------------------------------------
 * Code's references:
 * [1] AC 220V Heater Temperature PID and TRIAC control. (2018, avril 8). https://www.youtube.com/watch?v=P6mbBJDIvxI
 * [2] Build An Infrared Thermometer. (2020, mars 28). https://www.youtube.com/watch?v=UID87M-IKsg
 * [3] Measure current with ACS758 Current Sensor and LCD1602-I2C with Arduino. (2018, juin 20). https://www.youtube.com/watch?v=tug9wjCwDQA
 * [4] VCNL4000 Entfernungssensor, Helligkeitssensor am Arduino. (2014, décembre 16). https://www.youtube.com/watch?v=Rp5NcXffsBE 
 * ***
 * [5] analogRead()—Arduino Reference. (s. d.). Consulté 13 août 2020, à l’adresse https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/
 * [6] Arduino—WireBegin. (s. d.). Consulté 14 août 2020, à l’adresse https://www.arduino.cc/en/Reference/WireBegin
 * [7] analogWrite()—Arduino Reference. (s. d.). Consulté 14 août 2020, à l’adresse https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
---------------------------------------------------------------------------------------------------*/
