// Programme from Makerblog.at - Arduino & Co, VCNL4000 Sensor für Entfernung und Helligkeit am Arduino, https://www.youtube.com/watch?v=Rp5NcXffsBE


// Example sketch for talking to the VCNL4000 i2c proximity/light sensor
// Written by Adafruit! Public domain.
// From https://github.com/adafruit/Adafruit-VCNL40X0-PCB/blob/master/vcnl4000.ino
// To use: Connect VCC to 3.3-5V (5V is best if it is available), GND to
// ground, SCL to i2c clock (on classic arduinos, Analog 5), SDA
// to i2c data (on classic arduinos Analog 4). The 3.3v pin is
// an ouptut if you need 3.3V
// This sensor is 5V compliant so you can use it with 3.3 or 5V micros
 
// You can pick one up at the Adafruit shop: www.adafruit.com/products/466
 
#include <Wire.h>
 
// the i2c address
#define VCNL4000_ADDRESS 0x13
 
// commands and constants
#define VCNL4000_COMMAND 0x80
#define VCNL4000_PRODUCTID 0x81
#define VCNL4000_IRLED 0x83
#define VCNL4000_AMBIENTPARAMETER 0x84
#define VCNL4000_AMBIENTDATA 0x85
#define VCNL4000_PROXIMITYDATA 0x87
#define VCNL4000_SIGNALFREQ 0x89
#define VCNL4000_PROXINITYADJUST 0x8A
 
#define VCNL4000_3M125 0
#define VCNL4000_1M5625 1
#define VCNL4000_781K25 2
#define VCNL4000_390K625 3
 
#define VCNL4000_MEASUREAMBIENT 0x10
#define VCNL4000_MEASUREPROXIMITY 0x08
#define VCNL4000_AMBIENTREADY 0x40
#define VCNL4000_PROXIMITYREADY 0x20
 
 
// Winke-Pin leuchtet bei Näherung
const int wavePin = 13;
// Reset-Pin leuchtet bei Dunkelheit
const int resetPin = 12;
 
// Das Skript gibt Helligkeit und Entfernungswert auch an den seriellen
//  Monitor zurück, damit können die Grenzwerte gut bestimmt werden.
 
// Schwellenwert für Näherung (anpassen!)
const int minproximity = 2100;
// Schwellenwert für Helligkeit (anpassen!)
const int minambient = 2000;
// Aktueller Zustand der Winke-LED
int waveStatus = LOW;
 
void setup() {
  Serial.begin(9600);
 
  Serial.println("VCNL");
  Wire.begin();
 
  uint8_t rev = read8(VCNL4000_PRODUCTID);
 
  if ((rev & 0xF0) != 0x10) {
    Serial.println("Sensor not found :(");
    while (1);
  }
 
  write8(VCNL4000_IRLED, 20);        // set to 20 * 10mA = 200mA
  Serial.print("IR LED current = ");
  Serial.print(read8(VCNL4000_IRLED) * 10, DEC);
  Serial.println(" mA");
 
  //write8(VCNL4000_SIGNALFREQ, 3);
  Serial.print("Proximity measurement frequency = ");
  uint8_t freq = read8(VCNL4000_SIGNALFREQ);
  if (freq == VCNL4000_3M125) Serial.println("3.125 MHz");
  if (freq == VCNL4000_1M5625) Serial.println("1.5625 MHz");
  if (freq == VCNL4000_781K25) Serial.println("781.25 KHz");
  if (freq == VCNL4000_390K625) Serial.println("390.625 KHz");
 
  write8(VCNL4000_PROXINITYADJUST, 0x81);
  Serial.print("Proximity adjustment register = ");
  Serial.println(read8(VCNL4000_PROXINITYADJUST), HEX);
 
  // Die beiden LED-Pins auf OUTPUT setzen  
  pinMode(wavePin, OUTPUT);
  pinMode(resetPin, OUTPUT);
 
  // arrange for continuous conversion
  //write8(VCNL4000_AMBIENTPARAMETER, 0x89);
 
}
 
void loop() {
 
  // read ambient light!
  write8(VCNL4000_COMMAND, VCNL4000_MEASUREAMBIENT | VCNL4000_MEASUREPROXIMITY);
 
  while (1) {
    uint8_t result = read8(VCNL4000_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if ((result & VCNL4000_AMBIENTREADY)&&(result & VCNL4000_PROXIMITYREADY)) {
 
      Serial.print("Ambient = ");
      unsigned int ambient = read16(VCNL4000_AMBIENTDATA);
      Serial.print(ambient);
      Serial.print("\t\tProximity = ");
      unsigned int prox = read16(VCNL4000_PROXIMITYDATA);
      Serial.println(prox);
 
      // Wenn die Helligkeit größer als der Schwellenwert ist...
      if (ambient > minambient) {
        // dann Reset-Pin auf jeden Fall ausschalten       
        digitalWrite(resetPin, LOW);
        // Wenn dazu noch der Entfernungswert über die Schwelle steigt,
        // dann WinkePin-Status auf HIGH
        if (prox > minproximity) {
          waveStatus = HIGH;
        }
      }
      else
      {
        // Wenn Helligkeit kleiner als Schwellenwert, dann
        // ResetPin einschalten, WinkePin-Status auf LOW und eine Sekunde warten
        digitalWrite(resetPin, HIGH);
        waveStatus = LOW;
        delay(1000);
      }  
      break;
    }
    delay(10);
  }
 
  // WinkePin-Status ans wavePin schicken
  digitalWrite(wavePin, waveStatus);
 
  delay(50);
}
 
uint16_t readProximity() {
  write8(VCNL4000_COMMAND, VCNL4000_MEASUREPROXIMITY);
  while (1) {
    uint8_t result = read8(VCNL4000_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if (result & VCNL4000_PROXIMITYREADY) {
      return read16(VCNL4000_PROXIMITYDATA);
    }
    delay(1);
  }
}
 
// Read 1 byte from the VCNL4000 at 'address'
uint8_t read8(uint8_t address)
{
  uint8_t data;
 
  Wire.beginTransmission(VCNL4000_ADDRESS);
#if ARDUINO >= 100
  Wire.write(address);
#else
  Wire.send(address);
#endif
  Wire.endTransmission();
 
  delayMicroseconds(170);  // delay required
 
  Wire.requestFrom(VCNL4000_ADDRESS, 1);
  while(!Wire.available());
 
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}
 
// Read 2 byte from the VCNL4000 at 'address'
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
 
// write 1 byte
void write8(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(VCNL4000_ADDRESS);
#if ARDUINO >= 100
  Wire.write(address);
  Wire.write(data);  
#else
  Wire.send(address);
  Wire.send(data);  
#endif
  Wire.endTransmission();
}
