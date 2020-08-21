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
 *  IN                → SSR pin 2
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
 *  2               → IN current sensor → heating elements
 *  **************************************************************************************************
 ---------------------------------------------------------------------------------------------------*/

// Outside librairies used in this code ( see https://www.arduino.cc/reference/en/libraries/ )

#include <Wire.h> // to communicate with I2C / TWI devices ( MEGA → pins : 20 (SDA), 21 (SCL) )
#include <Adafruit_MLX90614.h> // to read the temperature from the sensor
#include <PID.h> // to control a PWN pin depending on a reading (proportional–integral–derivative controller)
#include <Robojax_AllegroACS_Current_Sensor.h> // to read the current from sensor



// Constants and variables's definition

// MEGA's inputs
const int VIOUT_CURRENT_SENSOR_PIN = A0;
const int SDA_PIN = 20;
const int SCL_PIN = 21;

// MEGA's outputs
const int SSR_PULSE_PIN = 2;

// MEGA's external interrupt pin (ref [8])
const int INTERRUPT_PIN = SDA_PIN; // seem to be a bad idea, how can I associate the address's value?

// PID constants and object
const double TEMP_SETPOINT = 160; // °C
const double TEMP_RANGE =  0.03125; // %
const double KP = 1; // proportional's coefficient
const double KI = 0; // integral's coefficient 
const double KD = 0; // derivative's coefficient
PID temp_PID( TEMP_SETPOINT, TEMP_RANGE, KP, KI, KD );

// PID variable
double temp_correction; // ratio

// Temperature constant and object
float TEMP_ERROR = 8; // °C , rectification of the temperature because of the physical montage. Error determined with experimental test and comparison.
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Temperature variables
float ambiantTemp; // °C
float headTemp; // °C

// Current constants and object (ref [3])
const float VCC = 5.0; // Volts
const int CURRENT_SENSOR_MODEL = 3; // ACS758LCB-050B
/* Reference number used in CURRENT_SENSOR_MODEL in fonction of the sensor's model
      0 → ACS712ELCTR-05B
      1 → ACS712ELCTR-20A
      2 → ACS712ELCTR-30A
      
      3 → ACS758LCB-050B
      4 → ACS758LCB-050U
      5 → ACS758LCB-100B
      6 → ACS758LCB-100U
      7 → ACS758KCB-150B
      8 → ACS758KCB-150U
      9 → ACS758ECB-200B
      10 → ACS758ECB-200U 
      
      11 → ACS770x-050B      
      12 → ACS770x-050U
      13 → ACS770x-100B
      14 → ACS770x-100U
      15 → ACS770x-150B
      16 → ACS770x-150U
      17 → ACS770x-200B  
      18 → ACS770x-200U  
      
      19 → ACS732KLATR-20AB
      20 → ACS732KLATR-40AB
      21 → ACS732KLATR-65AB
      22 → ACS732KLATR-65AU
      23 → ACS732KLATR-75AB

      24 → ACS733KLATR-20AB
      25 → ACS733KLATR-40AB
      26 → ACS733KLATR-40AU
      27 → ACS733KLATR-65AU
*/
Robojax_AllegroACS_Current_Sensor ACS758( CURRENT_SENSOR_MODEL,VIOUT_CURRENT_SENSOR_PIN );

// Current variable
float currentRead;

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
volatile int revolution = 0; // complete rotation (volatile because of interrupt)
unsigned int proximity; // unknown unit
int oldTime = 0; // ms
int TimeIntervalForRev; // ms

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

  // Specified there is a interrupt in the programm and what are his conditions to happen
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), isr_RPM, RISING);
}



// Continuous communications of the system
void loop() {

  // Read the temperatures
  headTemp = TEMP_ERROR + mlx.readObjectTempC();
  ambiantTemp = mlx.readAmbientTempC();
  
  // Find the correction to apply in fonction of PWM pins's duty cycle → between 0 & 255 (ref [7])
  temp_correction = temp_PID.PWM_run(SSR_PULSE_PIN, headTemp);

  // Read the current
  currentRead = ACS758.getCurrent();

  // (Ref [4])
  // Turn off the interrupt to let the revolution number how it is while calculating RPM
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
  
  // Calculate the time for revolutions : 
  //(ms since MEGA is running)- (the beginning time of a rotation in function of running time)
  TimeIntervalForRev = millis() - oldTime; 

  // Calculate the RPM
  rpm = ( revolution / TimeIntervalForRev ) * 60000; // rev/ms * ( (60s/min) * (1000ms/s) ) → RPM
  
  // Define the starting time of a new cycle of rotations
  oldTime = millis();

  // Restart the revolution counter
  revolution = 0;

  // Restart the interrupt 
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), isr_RPM, RISING);

  // Display of all data in the serial monitor (table)
  
}



// Fonctions used in setup() and loop()

// Read 16 bits (2 bytes) of data from the proximity sensor (VCNL4000) at a specified addresse (Ref [4])
uint16_t read16(uint8_t address) { 
  
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



// Interrupt service routine (isr) to count revolutions → suspended the programm wherever he is up to
// and priorise the action to add a revolution to the counter when required
void isr_RPM() {
  revolution++;
}



/*---------------------------------------------------------------------------------------------------
 * Code's references:
 * [1] AC 220V Heater Temperature PID and TRIAC control. (2018, avril 8). https://www.youtube.com/watch?v=P6mbBJDIvxI
 * [2] Build An Infrared Thermometer. (2020, mars 28). https://www.youtube.com/watch?v=UID87M-IKsg
 * [3] Measuring 5A to 30A AC and DC current using ACS712 with Robojax Library—Robojax. (s. d.). Consulté 21 août 2020, à l’adresse https://robojax.com/learn/arduino/?vid=robojax_Alegro_ACS712_curren_sensor
 * [4] Arduino Tutorial : Tachometer (RPM Counter). (s. d.). Consulté 20 août 2020, à l’adresse https://www.youtube.com/watch?v=u2uJMJWsfsg&t=144s
 * ***
 * [5] analogRead()—Arduino Reference. (s. d.). Consulté 13 août 2020, à l’adresse https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/
 * [6] Arduino—WireBegin. (s. d.). Consulté 14 août 2020, à l’adresse https://www.arduino.cc/en/Reference/WireBegin
 * [7] analogWrite()—Arduino Reference. (s. d.). Consulté 14 août 2020, à l’adresse https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
 * [8] Thangavel, Pramoth. (2019, février 12). Arduino Interrupts Tutorial. Circuit Digest. https://circuitdigest.com/microcontroller-projects/arduino-interrupt-tutorial-with-examples
 ---------------------------------------------------------------------------------------------------*/
