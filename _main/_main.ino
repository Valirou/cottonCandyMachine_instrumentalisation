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
 *  Address (I2C) : none
 *  
 *  SEN0098 (ACS758LCB-050B)  → Arduino MEGA or intermediate bus
 *  VCC               → 5V bus
 *  GND               → GND bus
 *  VIOUT             → A0
 *  In AC circuit :
 *  IN                → SSR pin 2
 *  OUT               → heating element
 *  **************************************************************************************************
 *  Hall effect sensor
 *  from https://abra-electronics.com/sensors/sensors-magneto-en-2/a3144-hall-effect-sensor-a3144.html
 *  Used with magnets buy from : https://abra-electronics.com/science/physics-en/mag-n-03-3x3mm-neodymium-rare-earth-round-magnet-pack-of-20.html
 *  Address : none
 *  
 *  A3144     → Arduino MEGA or intermediate bus
 *  VIN       → 5V bus
 *  GND       → GND bus
 *  OUTPUT    → PWM 3
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
const int INTERRUPT_PIN = 3;

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
const byte PULSES_PER_REVOLUTION = 2;
// If the period between pulses is too high, or even if the pulses stopped, then we would get stuck showing the
// last value instead of a 0. Because of this we are going to set a limit for the maximum period allowed.
// If the period is above this value, the RPM will show as 0.
// The higher the set value, the longer lag/delay will have to sense that pulses stopped, but it will allow readings
// at very low RPM.
// Setting a low value is going to allow the detection of stop situations faster, but it will prevent having low RPM readings.
// The unit is in microseconds.
const unsigned long ZERO_TIMEOUT = 100000;  // For high response time, a good value would be 100000.
                                           // For reading very low RPM, a good value would be 300000.
const byte NUM_READING = 2;  // Number of samples for smoothing. The higher, the more smoothing, but it's going to
                             // react slower to changes. 1 = no smoothing. Default: 2.

// Revolution variable
volatile unsigned long LastTimeWeMeasured;  // Stores the last time we measured a pulse so we can calculate the period.
volatile unsigned long PeriodBetweenPulses = ZERO_TIMEOUT + 1000;  // Stores the period between pulses in microseconds.
                       // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
volatile unsigned long PeriodAverage = ZERO_TIMEOUT + 1000;  // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.
                       // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
unsigned long FrequencyRaw;  // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.
unsigned long FrequencyReal;  // Frequency without decimals.
unsigned long rpm;  // Raw RPM without any processing.
unsigned int PulseCounter = 1;  // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.

unsigned long PeriodSum; // Stores the summation of all the periods to do the average.

unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;  // Stores the last time we measure a pulse in that cycle.
                                    // We need a variable with a value that is not going to be affected by the interrupt
                                    // because we are going to do math and functions that are going to mess up if the values
                                    // changes in the middle of the cycle.
unsigned long CurrentMicros = micros();  // Stores the micros in that cycle.
                                         // We need a variable with a value that is not going to be affected by the interrupt
                                         // because we are going to do math and functions that are going to mess up if the values
                                         // changes in the middle of the cycle.

// We get the RPM by measuring the time between 2 or more pulses so the following will set how many pulses to
// take before calculating the RPM. 1 would be the minimum giving a result every pulse, which would feel very responsive
// even at very low speeds but also is going to be less accurate at higher speeds.
// With a value around 10 you will get a very accurate result at high speeds, but readings at lower speeds are going to be
// farther from eachother making it less "real time" at those speeds.
// There's a function that will set the value depending on the speed so this is done automatically.
unsigned int AmountOfReadings = 1;

unsigned int ZeroDebouncingExtra;  // Stores the extra value added to the ZERO_TIMEOUT to debounce it.
                                   // The ZERO_TIMEOUT needs debouncing so when the value is close to the threshold it
                                   // doesn't jump from 0 to the value. This extra value changes the threshold a little
                                   // when we show a 0.
unsigned long readings[NUM_READING];  // The input.
unsigned long readIndex;  // The index of the current reading.
unsigned long total;  // The running total.
unsigned long average;  // The RPM value after applying the smoothing.

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

  // Give time to the hall effet sensor to get enough micros() → do not divided by negatives values
  delay(1000);
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
  // The following is going to store the two values that might change in the middle of the cycle.
  // We are going to do math and functions with those values and they can create glitches if they change in the
  // middle of the cycle.
  LastTimeCycleMeasure = LastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
  CurrentMicros = micros();  // Store the micros() in a variable.

  // CurrentMicros should always be higher than LastTimeWeMeasured, but in rare occasions that's not true.
  // I'm not sure why this happens, but my solution is to compare both and if CurrentMicros is lower than
  // LastTimeCycleMeasure I set it as the CurrentMicros.
  // The need of fixing this is that we later use this information to see if pulses stopped.
  if(CurrentMicros < LastTimeCycleMeasure){
    LastTimeCycleMeasure = CurrentMicros;
  }

  // Calculate the frequency:
  FrequencyRaw = 10000000000 / PeriodAverage;  // Calculate the frequency using the period between pulses.

  // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
  if(PeriodBetweenPulses > ZERO_TIMEOUT - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZERO_TIMEOUT - ZeroDebouncingExtra){  
    // If the pulses are too far apart that we reached the timeout for zero:
    FrequencyRaw = 0;  // Set frequency as 0.
    ZeroDebouncingExtra = 2000;  // Change the threshold a little so it doesn't bounce.
  }
  else{
    ZeroDebouncingExtra = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }
  FrequencyReal = FrequencyRaw / 10000;  // Get frequency without decimals.
                                          // This is not used to calculate RPM but we remove the decimals just in case
                                          // you want to print it.

  // Calculate the RPM:
  rpm = FrequencyRaw / PULSES_PER_REVOLUTION * 60;  // Frequency divided by amount of pulses per revolution multiply by
                                                  // 60 seconds to get minutes.
  rpm = rpm / 10000;  // Remove the decimals.

  // Smoothing RPM:
  total = total - readings[readIndex];  // Advance to the next position in the array.
  readings[readIndex] = rpm;  // Takes the value that we are going to smooth.
  total = total + readings[readIndex];  // Add the reading to the total.
  readIndex = readIndex + 1;  // Advance to the next position in the array.

  if (readIndex >= NUM_READING){
    // If we're at the end of the array:
    readIndex = 0;  // Reset array index.
  }
  
  // Calculate the average:
  average = total / NUM_READING;  // The average value it's the smoothed result.

  // Display of all data in the serial monitor (table)
  
}



// Fonctions used in setup() and loop()

// Interrupt service routine (isr) to calculate the period between pulses → suspended 
// the program wherever he is up to and priorise/run this fonction (Ref [4])
void isr_RPM() {
  PeriodBetweenPulses = micros() - LastTimeWeMeasured;  // Current "micros" minus the old "micros" when the last pulse happens.
                                                        // This will result with the period (microseconds) between both pulses.
                                                        // The way is made, the overflow of the "micros" is not going to cause any issue.

  LastTimeWeMeasured = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.

  if(PulseCounter >= AmountOfReadings){
    // If counter for amount of readings reach the set limit:
    PeriodAverage = PeriodSum / AmountOfReadings;  // Calculate the final period dividing the sum of all readings by the
                                                   // amount of readings to get the average.
    PulseCounter = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSum = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.

    // Change the amount of readings depending on the period between pulses.
    // To be very responsive, ideally we should read every pulse. The problem is that at higher speeds the period gets
    // too low decreasing the accuracy. To get more accurate readings at higher speeds we should get multiple pulses and
    // average the period, but if we do that at lower speeds then we would have readings too far apart (laggy or sluggish).
    // To have both advantages at different speeds, we will change the amount of readings depending on the period between pulses.
    // Remap period to the amount of readings:
    int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
    
    // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
    // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
    // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
    // 4th and 5th values are the amount of readings range.
    RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
    AmountOfReadings = RemapedAmountOfReadings;  // Set amount of readings as the remaped value.
  }
  else{
    PulseCounter++;  // Increase the counter for amount of readings by 1.
    PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
  }
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
