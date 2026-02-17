#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <SD.h>

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1

// Display Setup
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

ICM_20948_I2C ICM;

// Global Variables
int aState;
int aLastState;  
int counter = 1; // Start op 1
int buttonPin = 1; 
int rotaryPinA = 31;
int rotaryPinB = 32;
int rotaryButtonPin = 30;
int buttonRead;
int previous = LOW;
bool initialized = false;
bool bCW;
unsigned long lastTime = 0;
unsigned long debounce = 200UL;
const int chipSelect = BUILTIN_SDCARD;

// NEW: Timer for the display to prevent lag
unsigned long lastDisplayTime = 0;

// Storage for calculated Euler angles
double roll = 0.0, pitch = 0.0, yaw = 0.0;
double roll0 = 0.0, pitch0 = 0.0, yaw0 = 0.0;
double rollZ = 0.0, pitchZ = 0.0, yawZ = 0.0;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(rotaryPinA, INPUT_PULLUP);
  pinMode(rotaryPinB, INPUT_PULLUP);
  pinMode(rotaryButtonPin, INPUT_PULLUP);
  SERIAL_PORT.begin(115200);
  
  aLastState = digitalRead(rotaryPinA);
  // Initialize Display
  u8g2.begin();
  
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  // Initialize Sensor
  ICM.begin(WIRE_PORT, AD0_VAL);
  
  if (ICM.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.println("Trying again...");
    delay(500);
  } else {
    initialized = true;
  }

  // --- DMP CONFIGURATION ---
  bool success = true;
  success &= (ICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  // Set rate to 0 (Max speed)
  success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (ICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (ICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (ICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (ICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success) {
    SERIAL_PORT.println(F("DMP enabled!"));
  } else {
    SERIAL_PORT.println(F("Enable DMP failed!"));
  }
  // Let op: SD kaart logica heb ik laten staan, maar zorg dat de kaart erin zit of comment dit uit als het hangt
  File logFile = SD.open("/TEST.csv", FILE_WRITE);
}

void loop() {
  // --- Button Logic (Reset Zero) ---
  buttonRead = digitalRead(buttonPin);
  if (buttonRead == HIGH && previous == LOW && millis() - lastTime > debounce) {
    lastTime = millis();
    roll0 = roll;
    pitch0 = pitch;
    yaw0 = yaw;
  }
  previous = buttonRead;

  // --- Rotary Encoder Logic ---
  aState = digitalRead(rotaryPinA);
  if (aState != aLastState){ 
    if (digitalRead(rotaryPinB) != aState) { 
      // Met de klok mee (CW)
      counter++;
    } else { 
      // Tegen de klok in (CCW)
      counter--;
    }

    // --- HIER ZIT DE FIX ---
    // We laten hem doorlopen tot 9 (omdat 7 -> 8 -> 9 reset naar 1)
    if (counter > 8) {
      counter = 1;
    }
    // We laten hem doorlopen tot -1 (omdat 1 -> 0 -> -1 reset naar 7)
    if (counter < 0) {
      counter = 7;
    }
    
    Serial.print("Encoder Positie: ");
    Serial.println(counter);
  }
  aLastState = aState; 

  // ... De rest van je DMP en Display code blijft hetzelfde ...
  // --- DMP Data Read & Math ---
  icm_20948_DMP_data_t data;
  ICM.readDMPdataFromFIFO(&data);
  ICM.getAGMT();

  if ((ICM.status == ICM_20948_Stat_Ok) || (ICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double qw = q0;
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      roll = atan2(t0, t1) * 180.0 / PI;
      rollZ = roll - roll0; 

      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2) * 180.0 / PI;
      pitchZ = pitch - pitch0;
      
      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      yaw = atan2(t3, t4) * 180.0 / PI;
      yawZ = yaw - yaw0;
    }
  }

  // --- Display Logic (THROTTLED) ---
  if (millis() - lastDisplayTime > 50) { 
    lastDisplayTime = millis();
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x7_tr);
    
    if(counter == 1){
      u8g2.setCursor(5, 10);
      u8g2.print("Roll: "); u8g2.print(rollZ);
      u8g2.setCursor(5, 20);
      u8g2.print("Pitch: "); u8g2.print(pitchZ);
      u8g2.setCursor(5, 30);
      u8g2.print("Yaw: "); u8g2.print(yawZ);
    }
    else if(counter == 3){
      u8g2.setCursor(5, 10);
      u8g2.print("Acc X: "); u8g2.print(ICM.agmt.acc.axes.x);
      u8g2.setCursor(5, 20);
      u8g2.print("Acc Y: "); u8g2.print(ICM.agmt.acc.axes.y);
      u8g2.setCursor(5, 30);
      u8g2.print("Acc Z: "); u8g2.print(ICM.agmt.acc.axes.z);
    }
    else if(counter == 5){
      u8g2.setCursor(5, 10);
      u8g2.print("Gyro X: "); u8g2.print(ICM.agmt.gyr.axes.x);
      u8g2.setCursor(5, 20);
      u8g2.print("Gyro Y: "); u8g2.print(ICM.agmt.gyr.axes.y);
      u8g2.setCursor(5, 30);
      u8g2.print("Gyro Z: "); u8g2.print(ICM.agmt.gyr.axes.z);
    }
    else if(counter == 7){
      u8g2.setCursor(5, 10);
      u8g2.print("Mag X: "); u8g2.print(ICM.agmt.mag.axes.x);
      u8g2.setCursor(5, 20);
      u8g2.print("Mag Y: "); u8g2.print(ICM.agmt.mag.axes.y);
      u8g2.setCursor(5, 30);
      u8g2.print("Mag Z: "); u8g2.print(ICM.agmt.mag.axes.z);
    }
    // Bij counter == 0, 2, 4, 6, 8 blijft het scherm leeg (tussenstapjes)

    u8g2.sendBuffer();
  }
}