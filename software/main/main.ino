#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <SD.h>

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1

// --- DISPLAY SETUP ---
// Gebruik NONAME voor correcte weergave (geen strepen)
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

ICM_20948_I2C ICM;

// --- Global Variables ---
const int buttonPin = 1;         
const int rotaryPinA = 31;
const int rotaryPinB = 32;
const int rotaryButtonPin = 30; 

// Encoder & Button Logic
int aState;
int aLastState;  
int rotaryButtonState;
int lastRotaryButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Menu Logic
int menuIndex = 0; 
const int maxMenuItems = 4;
const char* menuItems[] = {"Angles", "Accel", "Gyro", "Magnet"};

// State Machine
enum AppState { STATE_MENU, STATE_DATA };
AppState currentState = STATE_MENU;

// Sensor Data
double roll = 0.0, pitch = 0.0, yaw = 0.0;
double roll0 = 0.0, pitch0 = 0.0, yaw0 = 0.0;
double rollZ = 0.0, pitchZ = 0.0, yawZ = 0.0;

unsigned long lastDisplayTime = 0;

// Zero Reset Button Logic
int buttonRead, previous = LOW;
unsigned long lastTime = 0;
unsigned long debounce = 200UL;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(rotaryPinA, INPUT_PULLUP);
  pinMode(rotaryPinB, INPUT_PULLUP);
  pinMode(rotaryButtonPin, INPUT_PULLUP);
  
  SERIAL_PORT.begin(115200);
  
  // Lees de startwaarde van Pin A
  aLastState = digitalRead(rotaryPinA);
  
  u8g2.begin();
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  ICM.begin(WIRE_PORT, AD0_VAL);
  if (ICM.status != ICM_20948_Stat_Ok) SERIAL_PORT.println("Sensor error");

  // DMP Setup
  ICM.initializeDMP();
  ICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
  ICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0);
  ICM.enableFIFO();
  ICM.enableDMP();
  ICM.resetDMP();
  ICM.resetFIFO();
}

void loop() {
  // Start ICM loop
  icm_20948_DMP_data_t data;
  ICM.readDMPdataFromFIFO(&data);
  ICM.getAGMT(); 

  if ((ICM.status == ICM_20948_Stat_Ok) || (ICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double qw = q0; double qx = q2; double qy = q1; double qz = -q3;

      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      roll = atan2(t0, t1) * 180.0 / PI;
      rollZ = roll - roll0; 

      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2; t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2) * 180.0 / PI;
      pitchZ = pitch - pitch0;
      
      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      yaw = atan2(t3, t4) * 180.0 / PI;
      yawZ = yaw - yaw0;
    }
  }
  // End ICM loop
  // Input H
  buttonRead = digitalRead(buttonPin);
  if (buttonRead == HIGH && previous == LOW && millis() - lastTime > debounce) {
    lastTime = millis();
    roll0 = roll; pitch0 = pitch; yaw0 = yaw;
  }
  previous = buttonRead;

  // --- ROTARY ENCODER (1-stap fix) ---
  aState = digitalRead(rotaryPinA);
  
  // We reageren nu ALLEEN als de status van HOOG naar LAAG gaat (Falling edge)
  // Dit halveert het aantal pulsen en lost het 2-stappen probleem op.
  if (aState == LOW && aLastState == HIGH && currentState == STATE_MENU) {
    
    // Lees Pin B om de richting te bepalen
    // Als B ook LOW is, draaien we de ene kant op, anders de andere kant.
    if (digitalRead(rotaryPinB) == LOW) {
      menuIndex++; 
    } else {
      menuIndex--;
    }
    
    // Wrap around logic
    if (menuIndex >= maxMenuItems) menuIndex = 0;
    if (menuIndex < 0) menuIndex = maxMenuItems - 1;
  }
  aLastState = aState; // Update de status voor de volgende loop

  // --- ROTARY BUTTON ---
  int reading = digitalRead(rotaryButtonPin);
  if (reading != lastRotaryButtonState) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != rotaryButtonState) {
      rotaryButtonState = reading;
      if (rotaryButtonState == LOW) {
        // Toggle tussen Menu en Data
        if (currentState == STATE_MENU) currentState = STATE_DATA;
        else currentState = STATE_MENU;
      }
    }
  }
  lastRotaryButtonState = reading;

  // --- 3. DISPLAY ---
  if (millis() - lastDisplayTime > 50) { 
    lastDisplayTime = millis();
    u8g2.clearBuffer();
    
    if (currentState == STATE_MENU) {
      drawMenu();
    } else {
      drawData();
    }
    u8g2.sendBuffer();
  }
}



void drawMenu() {
  u8g2.setFont(u8g2_font_5x7_tr);

  int prevIndex = (menuIndex - 1 + maxMenuItems) % maxMenuItems;
  int nextIndex = (menuIndex + 1) % maxMenuItems;

  // Vorige
  u8g2.setCursor(12, 8); 
  u8g2.print(menuItems[prevIndex]);

  // Huidige (met pijl)
  u8g2.setCursor(0, 19); 
  u8g2.print("> ");
  u8g2.print(menuItems[menuIndex]);

  // Volgende
  u8g2.setCursor(12, 30); 
  u8g2.print(menuItems[nextIndex]);
}

void drawData() {
  u8g2.setFont(u8g2_font_5x7_tr);
  
  double v1, v2, v3;
  const char *l1, *l2, *l3;

  switch(menuIndex) {
    case 0: l1="Roll : "; v1=rollZ; l2="Pitch: "; v2=pitchZ; l3="Yaw  : "; v3=yawZ; break;
    case 1: l1="Acc X: "; v1=ICM.agmt.acc.axes.x; l2="Acc Y: "; v2=ICM.agmt.acc.axes.y; l3="Acc Z: "; v3=ICM.agmt.acc.axes.z; break;
    case 2: l1="Gyr X: "; v1=ICM.agmt.gyr.axes.x; l2="Gyr Y: "; v2=ICM.agmt.gyr.axes.y; l3="Gyr Z: "; v3=ICM.agmt.gyr.axes.z; break;
    case 3: l1="Mag X: "; v1=ICM.agmt.mag.axes.x; l2="Mag Y: "; v2=ICM.agmt.mag.axes.y; l3="Mag Z: "; v3=ICM.agmt.mag.axes.z; break;
  }

  // Data netjes uitgelijnd op 5,10 / 5,20 / 5,30
  u8g2.setCursor(5, 10); u8g2.print(l1); u8g2.print(v1);
  u8g2.setCursor(5, 20); u8g2.print(l2); u8g2.print(v2);
  u8g2.setCursor(5, 30); u8g2.print(l3); u8g2.print(v3);
}