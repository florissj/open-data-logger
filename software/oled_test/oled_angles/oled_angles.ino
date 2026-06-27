#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ICM_20948.h>

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1

// Display Setup
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

ICM_20948_I2C ICM;

// Global Variables
int counter = 1;
int buttonPin = 1; 
int buttonRead;
int previous = LOW;
bool initialized = false;
unsigned long lastTime = 0;
unsigned long debounce = 200UL;

// NEW: Timer for the display to prevent lag
unsigned long lastDisplayTime = 0;

// Storage for calculated Euler angles
double roll = 0.0, pitch = 0.0, yaw = 0.0;

void setup() {
  pinMode(buttonPin, INPUT);
  
  SERIAL_PORT.begin(115200);
  
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
}

void loop() {
  // --- Button Logic ---
  buttonRead = digitalRead(buttonPin);
  if (buttonRead == HIGH && previous == LOW && millis() - lastTime > debounce) {
    if (counter < 4) {
      counter++;
    } else {
      counter = 1;
    }
    lastTime = millis();
  }
  previous = buttonRead; 

  // --- DMP Data Read & Math ---
  // We read this EVERY loop to keep the sensor memory (FIFO) empty
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

      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2) * 180.0 / PI;

      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      yaw = atan2(t3, t4) * 180.0 / PI;
    }
  }

  // --- Display Logic (THROTTLED) ---
  // Only update display every 50ms (20 FPS). 
  // This allows the sensor loop above to run fast enough to not lag.
  if (millis() - lastDisplayTime > 50) { 
    lastDisplayTime = millis();
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenR08_tr);
    if(counter == 1){
    u8g2.setCursor(5, 10);
    u8g2.print("Roll: "); 
    u8g2.print(roll);

    u8g2.setCursor(5, 21);
    u8g2.print("Pitch: "); 
    u8g2.print(pitch);

    u8g2.setCursor(5, 32);
    u8g2.print("Yaw: "); 
    u8g2.print(yaw);
    }
    else if(counter == 2){
    u8g2.setCursor(5, 10);
    u8g2.print("Acc X: "); 
    u8g2.print(ICM.agmt.acc.axes.x);

    u8g2.setCursor(5, 21);
    u8g2.print("Acc Y: "); 
    u8g2.print(ICM.agmt.acc.axes.y);

    u8g2.setCursor(5, 32);
    u8g2.print("Acc Z: "); 
    u8g2.print(ICM.agmt.acc.axes.z);
    }
    else if(counter == 3){
    u8g2.setCursor(5, 10);
    u8g2.print("Gyro X: "); 
    u8g2.print(ICM.agmt.gyr.axes.x);

    u8g2.setCursor(5, 21);
    u8g2.print("Gyro Y: "); 
    u8g2.print(ICM.agmt.gyr.axes.y);

    u8g2.setCursor(5, 32);
    u8g2.print("Gyro Z: "); 
    u8g2.print(ICM.agmt.gyr.axes.z);
    }
    else if(counter == 4){
    u8g2.setCursor(5, 10);
    u8g2.print("Mag X: "); 
    u8g2.print(ICM.agmt.mag.axes.x);

    u8g2.setCursor(5, 21);
    u8g2.print("Mag Y: "); 
    u8g2.print(ICM.agmt.mag.axes.y);

    u8g2.setCursor(5, 32);
    u8g2.print("Mag Z: "); 
    u8g2.print(ICM.agmt.mag.axes.z);
    }

    u8g2.sendBuffer();
  }
}