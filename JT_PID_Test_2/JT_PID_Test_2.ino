// Create by James T Teasdale

// Components pins
#define Light_LED_pin 3
#define Light_sensorPin A0

// Potentionmeter Pins
#define P_pin A1
#define I_pin A2
#define D_pin A3

// PID settings and gains
const int Light_SetPoint = 100;
//const int Light_OutputMin = 0;
//const int Light_OutputMax = 255;
double Light_Kp = 0.1;
double Light_Ki = 0.0;
double Light_Kd = 0.0;

//const int setPoint = 150;
//const int outputMin = 0;
//const int outputMax = 255;
//const double Kp = 0;
//const double Ki = 0;
//const double Kd = 0;

// PID memories
unsigned long Light_lastTime;
double Light_lastError;
double Light_lastArea;

double PID(int value, double setPoint, double &lastError, double &lastArea, unsigned long &lastTime, double Kp, double Ki, double Kd, int maxOutput = 255, int minOutput = 0) {
  // Asserts
  if (maxOutput > 255) {
    maxOutput = 255;
  }
  if (minOutput < 0) {
    minOutput = 0;
  }
  if (minOutput > maxOutput) {
    minOutput = maxOutput;
  }
  
  // Store actual time
  unsigned long thisTime = millis();

  // Assert
  if (lastTime > thisTime) {
    thisTime = lastTime;
  }

  // Get signal error
  long error = setPoint - value;
//  Serial.print("| Error = " + String(error) + " |");

  // Calculate signal correction
  double P_Gain = error * Kp;
  double I_Gain = integrate(lastError, error, lastArea, lastTime, thisTime)*Ki;
  double D_Gain = differentiate(Light_lastError, error, lastTime, thisTime)*Kd;
  double correction = P_Gain + I_Gain + D_Gain;

//  Serial.print("| P_Gain = " + String(P_Gain) + " |");
//  Serial.print("| I_Gain = " + String(I_Gain) + " |");
//  Serial.print("| D_Gain = " + String(D_Gain) + " |");

  // Check output limitations
  if (correction > maxOutput) {
    correction = maxOutput;
  }
  else if (correction < minOutput) {
    correction = minOutput;
  }
//  Serial.print("| Correction = " + String(correction) + " |");
  
  // Store error and processing time
  lastError = correction;
  lastTime = thisTime;
  
  return correction;
}

bool isAtSetPoint(double value, double setPoint, double range, int indicatorPin = -1) {
  // Asserts
  if (range < 0) {
    range = 0;
  }
  else if (range > 1) {
    range = 1;
  }

  double minValue = setPoint * (1 - range);
  double maxValue = setPoint * (1 + range);

  bool isSetPointReached = (minValue <= value ) && (value <= maxValue);

//  Serial.print("| Reached = " + String(isSetPointReached) + " |");
  
  if (indicatorPin != -1) {
    if (isSetPointReached) {
      digitalWrite(indicatorPin, HIGH);
    }
    else {
      digitalWrite(indicatorPin, LOW);
    }
  }
  
  return isSetPointReached;
  
}

void setup() {
  pinMode(P_pin, INPUT);
  pinMode(I_pin, INPUT);
  pinMode(D_pin, INPUT);
  
  pinMode(Light_sensorPin, INPUT);
  pinMode(Light_LED_pin, OUTPUT);
  pinMode(13, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  //Light_Kp = analogRead(P_pin) * 0.01;
  //Light_Ki = analogRead(I_pin) * 0.0001;
  //Light_Kd = analogRead(D_pin) * 0.01;
  
  //Serial.print("| Light_lastError = " + String(Light_lastError) + " |");
  //Serial.print("| Light_lastArea = " + String(Light_lastArea) + " |");
  
  // Read light sensor value
  long Light_sensorValue = analogRead(Light_sensorPin);
  Serial.print(String(Light_sensorValue)); //"| Measured light = " + String(Light_sensorValue) + " |");

  //Light_sensorValue = ajustSensorValue(Light_sensorValue, 0, 1024, 0, 255);
  
  double Light_correction = PID(Light_sensorValue, Light_SetPoint, Light_lastError, Light_lastArea, Light_lastTime, Light_Kp, Light_Ki, Light_Kd);
  //Serial.print("| Correction = " + String(Light_correction) + " |");
  analogWrite(Light_LED_pin, Light_correction);
  isAtSetPoint(Light_sensorValue, Light_SetPoint, .1, 13);

  Serial.println();
  //delay(250);
}

double integrate(double &lastValue, double thisValue, double &lastArea, unsigned long lastTime, unsigned long thisTime) {
  // Asserts
  if (lastTime > thisTime) {
    thisTime = lastTime;
  }
  
//  Serial.print("| lastValue = " + String(lastValue) + " |");
//  Serial.print("| thisValue = " + String(thisValue) + " |");
//  Serial.print("| lastTime = " + String(lastTime) + " |");
//  Serial.print("| thisTime = " + String(thisTime) + " |");
  
  // Integrate by approximating the area under the curve with the erea of a trapeze since the time between processing is REALLY short
  double thisArea = (thisValue + lastValue) * (thisTime - lastTime) / 2.0;
//  Serial.print("| thisArea = " + String(thisArea) + " |");
  if (isnan(thisArea)) {
    thisArea = 0;
  }
  
  // Calculate the total area
  double totalArea = lastArea + thisArea;
//  Serial.print("| totalArea = " + String(totalArea) + " |");
  
  // Store new total area for next integration
  lastArea = totalArea;
  lastValue = thisValue;
  
  return totalArea;
}

double differentiate(double lastValue, double thisValue, unsigned long lastTime, unsigned long thisTime) {
  // Asserts
  if (lastTime > thisTime) {
    thisTime = lastTime;
  }
  
  // Differentiate by approximating the curve with a line since the time between processing is REALLY short
  return (thisValue - lastValue) / (thisTime - lastTime);
}

double ajustSensorValue(double value, int sensorMinValue, int sensorMaxValue, int outputMinValue, int outputMaxValue) {
  return interpolate(value, sensorMinValue, sensorMaxValue, outputMinValue, outputMaxValue);
}

double interpolate(double x, double x_min, double x_max, double y_min, double y_max) {
  double y = ((x-x_min) / (x_max-x_min)) * (y_max-y_min) + y_min;
  return y;
}
