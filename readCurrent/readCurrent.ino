// Programme from Robojax, Measure Current using Allegro ACS758 Current Sensor with LCD1602 (I2C) for Arduino, https://www.youtube.com/watch?v=tug9wjCwDQA

 * 
 * Arduino Sketch for Allegro ACS758 Current Sensor with LCD1602 & I2C module
 * this sensor can measure current at range of up to 200A
 * It operates with 3.3 or 5V
 * This video requires you to watch the following 2 videos before using this code:
 * 1- ACS758 Sensor https://www.youtube.com/watch?v=SiHfjzcqnU4
 * 2- LCD1602 with I2C https://www.youtube.com/watch?v=q9YC_GVHy5A
 * 
 * 
 * Written by Ahmad Shamshiri on Tuesday June 19,2018 at 20:40 at Ajax, Ontario, Canada
 * for Robojax.com
 * View the video instruction fro this code at https://youtu.be/tug9wjCwDQA
 * This code has been downloaded from Robojax.com
 */
#define VIN A0 // define the Arduino pin A0 as voltage input (V in)
const float VCC   = 5.0;// supply voltage 5V or 3.3V. If using PCB, set to 5V only.
const int model = 2;   // enter the model (see below)

float cutOffLimit = 1.00;// reading cutt off current. 1.00 is 1 Amper

/*
          "ACS758LCB-050B",// for model use 0
          "ACS758LCB-050U",// for model use 1
          "ACS758LCB-100B",// for model use 2
          "ACS758LCB-100U",// for model use 3
          "ACS758KCB-150B",// for model use 4
          "ACS758KCB-150U",// for model use 5
          "ACS758ECB-200B",// for model use 6
          "ACS758ECB-200U"// for model use  7   
sensitivity array is holding the sensitivy of the  ACS758
current sensors. Do not change.          
*/
float sensitivity[] ={
          40.0,// for ACS758LCB-050B
          60.0,// for ACS758LCB-050U
          20.0,// for ACS758LCB-100B
          40.0,// for ACS758LCB-100U
          13.3,// for ACS758KCB-150B
          16.7,// for ACS758KCB-150U
          10.0,// for ACS758ECB-200B
          20.0,// for ACS758ECB-200U     
         }; 

/*         
 *   quiescent Output voltage is factor for VCC that appears at output       
 *   when the current is zero. 
 *   for Bidirectional sensor it is 0.5 x VCC
 *   for Unidirectional sensor it is 0.12 x VCC
 *   for model ACS758LCB-050B, the B at the end represents Bidirectional (polarity doesn't matter)
 *   for model ACS758LCB-100U, the U at the end represents Unidirectional (polarity must match)
 *    Do not change.
 */
float quiescent_Output_voltage [] ={
          0.5,// for ACS758LCB-050B
          0.12,// for ACS758LCB-050U
          0.5,// for ACS758LCB-100B
          0.12,// for ACS758LCB-100U
          0.5,// for ACS758KCB-150B
          0.12,// for ACS758KCB-150U
          0.5,// for ACS758ECB-200B
          0.12,// for ACS758ECB-200U            
          };
const float FACTOR = sensitivity[model]/1000;// set sensitivity for selected model
const float QOV =   quiescent_Output_voltage [model] * VCC;// set quiescent Output voltage for selected model
float voltage;// internal variable for voltage
float cutOff = FACTOR/cutOffLimit;// convert current cut off to mV

// ======== start of LCD1602 with i2C settings
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
// ======= END of  LCD1602 with i2C settings
void setup() {
    //Robojax.com ACS758 Current Sensor 
    Serial.begin(9600);// initialize serial monitor
    Serial.println("Robojax Tutorial");
    Serial.println("ACS758 Current Sensor");
    Serial.println("with LCD1602 & I2C");    
    // initialize the LCD, 
    lcd.begin();   
  // Turn on the blacklight and print a message.
  lcd.backlight(); 
    lcd.clear();
  lcd.print("Robojax");
  lcd.setCursor (0,1); // go to start of 2nd line 
  lcd.print("ACS758 Current Sensor"); 
  delay(2000); 
  lcd.clear();  
}

void loop() {
  //Robojax code ACS758 with LCD1602 and I2C
  float voltage_raw =   (5.0 / 1023.0)* analogRead(VIN);// Read the voltage from sensor
  voltage =  voltage_raw - QOV + 0.007 ;// 0.007 is a value to make voltage zero when there is no current
  float current = voltage / FACTOR;
  if(abs(voltage) > cutOff ){
    Serial.print("V: ");
    Serial.print(voltage,3);// print voltage with 3 decimal places
    Serial.print("V, I: ");
    Serial.print(current,2); // print the current with 2 decimal places
    Serial.println("A");
  //start of loop Robojax code ACS758 with LCD1602 and I2C
  lcd.clear();
  lcd.setCursor (0,0); // set to line 1, char 0  
  lcd.print("Current: ");
  lcd.setCursor (9,0); // go to start of 2nd line
  lcd.print(current);
  lcd.setCursor (15,0); // go to start of 2nd line
  lcd.print("A");

  lcd.setCursor (0,1);    
  lcd.print("Sense V: ");
  lcd.setCursor (9,1); // go to start of 2nd line
  lcd.print(voltage);
  lcd.setCursor (15,1); // go to start of 2nd line
  lcd.print("V");   
  lcd.backlight();
 //end of loopcode Robojax code ACS758 with LCD1602 and I2C


    
  }else{
    Serial.println("No Current");
  lcd.clear();    
  lcd.setCursor (0,0);    
  lcd.print("No Current");    
  }
  delay(500);
}
