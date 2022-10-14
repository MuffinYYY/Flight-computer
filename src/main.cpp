#include <Arduino.h>

#include <Wire.h>
#include <SFE_BMP180.h>
#include <MPU6050.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <Servo.h>
#include <ArduinoJson.h>

SFE_BMP180 pressure;
MPU6050 mpu;
Adafruit_NeoPixel pixels(1, 3, NEO_GRB + NEO_KHZ800);
Servo xservo;
Servo yservo;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

int buzzer = 6;

double baseline;

StaticJsonDocument<200> doc;

double led (int color) {
  //For turning off LED
  if(color==0){  
    pixels.clear();
    for(int i=0; i<1; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();  
    }  
  }
    
  //For errors: RED color
  if(color==1){  
    pixels.clear();
    for(int i=0; i<1; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 150, 0));
    pixels.show();  
    delay(1000);
    }  
  }
  //For sucess init: GREEN color
  if(color==2){  
    pixels.clear();
    for(int i=0; i<1; i++) {
    pixels.setPixelColor(i, pixels.Color(150, 0, 0));
    pixels.show();  
    }  
  } 
  return 0;
}

int voltage(int delayTime){
// number of analog samples to take per reading
#define NUM_SAMPLES 10

int sum = 0;                    // sum of samples taken
unsigned char sample_count = 0; // current sample number
float voltage = 0.0;   

 // take a number of analog samples and add them up
    while (sample_count < NUM_SAMPLES) {
        sum += analogRead(A1);
        sample_count++;
        delay(delayTime);
    }
    // calculate the voltage
    // use 5.0 for a 5.0V ADC reference voltage
    // 5.015V is the calibrated reference voltage
    voltage = ((float)sum / (float)NUM_SAMPLES * 5.015) / 1024.0;
    // send voltage for display on Serial Monitor
    // voltage multiplied by 11 when using voltage divider that
    // divides by 11. 11.132 is the calibrated voltage divide
    // value
    doc["voltage"]=voltage * 11.132;
    serializeJson(doc, Serial);

    sample_count = 0;
    sum = 0;
    return 0;
}

void gyro() {
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw

  doc["pitch"]=pitch;
  doc["yaw"]=yaw;
  doc["roll"]=roll;

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}

// Getting altitude function
double getPressure()
{
  char status;//Curent status
  double T,P;//Variables for determining pressure

  // You must first get a temperature measurement to perform a pressure reading
 
  // Start a temperature measurement:
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
      }
    }
  }
  return 0; //Exit condition
}

void get_altitude(){
  double a,P;
  
  // Get a new pressure reading:

  P = getPressure();

  // Show the relative altitude difference between
  // the new reading and the baseline reading:

  a = pressure.altitude(P,baseline);

  doc["altitude"]=a;
  Serial.println();
  
  delay(1);
}


void setup() {
  delay(1000);
  Serial.begin(115200);
  pixels.begin();
  xservo.attach(10);
  yservo.attach(9);
  pinMode(buzzer, OUTPUT);
  while(!Serial){ 
      //Comment out if on battery 
      led(1); 
  }
  Serial.println("Test begining:");

    while(!pressure.begin())
  {
    Serial.println("Could not find a valid BMP180 sensor");
    led(1);
  } 
  Serial.println("BMP180 init success");
  delay(500);
  
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor");
    led(1);
  } 
  Serial.println("MPU6050 init success");
  delay(500);
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  
  Serial.println("Initializing SD card");

  Serial.println("Initialization success");

  voltage(10);
  led(0);

  xservo.write(90);
  delay(500);
  xservo.write(0);
  delay(500);
  
  yservo.write(90);
  delay(500);
  yservo.write(0);
  
  Serial.println("Computer is configured for flight");
  delay(2000);
  baseline = getPressure();

}

void loop() {
  get_altitude();
  led(2);
  voltage(1);
  gyro(); 
  
}