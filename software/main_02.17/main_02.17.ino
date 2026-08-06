#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SdConfigFile.h>

// Initialize Display
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
ICM_20948_I2C ICM;
SoftwareSerial ss(4, 3);
SdConfigFile configFile(BUILTIN_SDCARD);

// Define Pins
const int rotaryPinA = 31;
const int rotaryPinB = 32;
const int rotaryButtonPin = 30; // Select
const int backButtonPin = 29;    // Back
const int chipSelect = BUILTIN_SDCARD; // Teensy SDcard

const int maxFiles = 10;
int fileCounter = 0;

String fileList[maxFiles];
String configDir = "/config/";


// Define Menu States
enum StateID {
  STATE_MAIN_MENU,
  STATE_SENSOR_MENU,
  STATE_SETTINGS_MENU,
  STATE_VIEW_ANGLES,
  STATE_VIEW_ACCEL,
  STATE_VIEW_GYRO,
  STATE_VIEW_MAG,
  STATE_ACTION_ZERO_RESET,
  STATE_VIEW_CONFIG
};

StateID currentState = STATE_MAIN_MENU;

// Menu Structure
struct MenuItem {
  const char* label;
  StateID nextState;
};

const MenuItem mainMenu[] = {
  {"Sensors >", STATE_SENSOR_MENU},
  {"Instellingen >", STATE_SETTINGS_MENU}
};

const MenuItem sensorMenu[] = {
  {"Display Angles", STATE_VIEW_ANGLES},
  {"Display Accel",  STATE_VIEW_ACCEL},
  {"Display Gyro",   STATE_VIEW_GYRO},
  {"Display Magnet", STATE_VIEW_MAG}
};

const MenuItem settingsMenu[] = {
  {"Set Zero",    STATE_ACTION_ZERO_RESET},
  {"Info",        STATE_MAIN_MENU},
  {"Load Config", STATE_VIEW_CONFIG}
};

MenuItem configMenu[maxFiles];


// Navigation
const MenuItem* currentMenuPtr = mainMenu; 
int currentMenuLength = 2;               
int menuIndex = 0;                       

// --- HARDWARE VARIABELEN ---
int aState, aLastState;
int selectBtnState, lastSelectBtnState = HIGH;
int backBtnState, lastBackBtnState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
String mainIMU;

// Display Timer (TEGEN LAG)
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 0; // Update scherm elke 50ms (20 FPS)

// Sensor Data Variables
double roll = 0.0, pitch = 0.0, yaw = 0.0;
double roll0 = 0.0, pitch0 = 0.0, yaw0 = 0.0;
double rollZ = 0.0, pitchZ = 0.0, yawZ = 0.0;

void setup() {
  Serial.begin(115200);
  
  pinMode(rotaryPinA, INPUT_PULLUP);
  pinMode(rotaryPinB, INPUT_PULLUP);
  pinMode(rotaryButtonPin, INPUT_PULLUP);
  pinMode(backButtonPin, INPUT_PULLUP);
//  pinMode(chipSelect, OUTPUT);
  
  aLastState = digitalRead(rotaryPinA);

  if (SD.begin(chipSelect)){
    File folder = SD.open("/config/");
    listFiles(folder);
  }
  else{
    Serial.println("SD failed");
  }

  Wire.begin();
  Wire.setClock(400000); 
  u8g2.begin();
  
  ICM.begin(Wire, 1);
  ICM.initializeDMP();
  ICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);

  ICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0); 
  ICM.enableFIFO();
  ICM.enableDMP();
  ICM.resetDMP();
  ICM.resetFIFO();

}

void switchMenu(const MenuItem* newMenu, int length) {
  currentMenuPtr = newMenu;
  currentMenuLength = length;
  menuIndex = 0; 
}

void handleBackPress() {
  switch (currentState) {
    case STATE_VIEW_CONFIG:
      currentState = STATE_SETTINGS_MENU;
      switchMenu(settingsMenu, 3);
      break;
    case STATE_VIEW_ANGLES:
    case STATE_VIEW_ACCEL:
    case STATE_VIEW_GYRO:
    case STATE_VIEW_MAG:
      currentState = STATE_SENSOR_MENU;
      switchMenu(sensorMenu, 4); 
      break;
    case STATE_SENSOR_MENU:
    case STATE_SETTINGS_MENU:
      currentState = STATE_MAIN_MENU;
      switchMenu(mainMenu, 2);
      break;
    case STATE_MAIN_MENU:
      break;
  
  }
}

void handleSelectPress() {
  bool isMenu = (currentState == STATE_MAIN_MENU || currentState == STATE_SENSOR_MENU || currentState == STATE_SETTINGS_MENU || currentState == STATE_VIEW_CONFIG);
  
  if (isMenu) {
    if (currentState == STATE_VIEW_CONFIG){
      String selectedFile = fileList[menuIndex];
      Serial.print("Loading file: ");
      Serial.println(selectedFile);
      currentState = STATE_SETTINGS_MENU;
      switchMenu(settingsMenu, 3);
      applyConfig(selectedFile);
    }
    StateID target = currentMenuPtr[menuIndex].nextState;
    switch (target) {
      case STATE_SENSOR_MENU:
        currentState = STATE_SENSOR_MENU;
        switchMenu(sensorMenu, 4); 
        break;
      case STATE_SETTINGS_MENU:
        currentState = STATE_SETTINGS_MENU;
        switchMenu(settingsMenu, 3);
        break;
      case STATE_ACTION_ZERO_RESET:
        roll0 = roll; pitch0 = pitch; yaw0 = yaw;
        break;
      case STATE_VIEW_CONFIG:
        currentState = STATE_VIEW_CONFIG;
        switchMenu(configMenu, fileCounter);
      default:
        currentState = target; 
        break;
    }
  }
}

void applyConfig(String selectedFile) {
  String path = "/config/" + selectedFile;
  File file = SD.open(path.c_str());
  
  if (!file) {
    Serial.println("Kan configuratiebestand niet openen!");
    return;
  }

  // Lees het bestand regel voor regel
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim(); // Haal onzichtbare spaties en enters weg
    
    // Negeer lege regels of comments (zoals in de library)
    if (line.length() == 0 || line.startsWith("//") || line.startsWith("#")) {
      continue;
    }

    // Splits de regel bij het '=' teken
    int equalsIndex = line.indexOf('=');
    if (equalsIndex > 0) {
      String key = line.substring(0, equalsIndex);
      String value = line.substring(equalsIndex + 1);
      
      key.trim();
      value.trim();

      // Kijk of het de variabele is die we zoeken
      if (key == "mainIMU") {
        mainIMU = value;
        Serial.print("mainIMU ingeladen: ");
        Serial.println(mainIMU);
      }
      

    }
  }
  
  file.close(); // Vergeet niet af te sluiten
}

void listFiles(File dir) {
  while(true){
    File entry = dir.openNextFile();
    if (!entry){
      break;
    }
    if(fileCounter < maxFiles){
      fileList[fileCounter] = String(entry.name());
      configMenu[fileCounter].label = fileList[fileCounter].c_str();
      configMenu[fileCounter].nextState = STATE_VIEW_CONFIG;
      fileCounter++;
    }
    Serial.println(entry.name());
    entry.close();
  }
}

void getICMdata(){
  icm_20948_DMP_data_t data;
  
  ICM.getAGMT(); // Haal raw data op (Accel/Gyro/Mag)
  ICM.readDMPdataFromFIFO(&data); // Lees het eerste quaternion pakketje

  // Gebruik een WHILE loop om de hele buffer te legen!
  while ((ICM.status == ICM_20948_Stat_Ok) || (ICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
      
      // NaN Fix: Zorg dat sumSq nooit > 1.0 is
      double sumSq = (q1 * q1) + (q2 * q2) + (q3 * q3);
      if (sumSq > 1.0) sumSq = 1.0;
      double q0 = sqrt(1.0 - sumSq);

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
    // Lees het volgende pakketje om de while-loop draaiende te houden
    ICM.readDMPdataFromFIFO(&data);
  }
}

void loop() {
  // 
  getICMdata();

  while (ss.available() > 0){
    // get the byte data from the GPS
    byte gpsData = ss.read();
    Serial.write(gpsData);
  }

  // --- 2. NAVIGATIE: Scrollen ---
  bool isMenu = (currentState == STATE_MAIN_MENU || currentState == STATE_SENSOR_MENU || currentState == STATE_SETTINGS_MENU || currentState == STATE_VIEW_CONFIG);
  
  aState = digitalRead(rotaryPinA);
  if (isMenu && aState == LOW && aLastState == HIGH) { 
    if (digitalRead(rotaryPinB) == LOW) {
        menuIndex++; 
    } else {
        menuIndex--; 
    }
    
    if (menuIndex >= currentMenuLength) menuIndex = 0;
    if (menuIndex < 0) menuIndex = currentMenuLength - 1;
  }
  aLastState = aState;

  // --- 3. KNOPPEN ---
  int readingSelect = digitalRead(rotaryButtonPin);
  if (readingSelect != lastSelectBtnState && readingSelect == LOW) { 
      if (millis() - lastDebounceTime > debounceDelay) {
         handleSelectPress();
         lastDebounceTime = millis();
      }
  }
  lastSelectBtnState = readingSelect;

  int readingBack = digitalRead(backButtonPin);
  if (readingBack != lastBackBtnState && readingBack == LOW) { 
      delay(50); 
      if(digitalRead(backButtonPin) == LOW) handleBackPress();
  }
  lastBackBtnState = readingBack;

  // --- 4. DISPLAY TEKENEN (Met Timer!) ---
  if (millis() - lastDisplayTime > displayInterval) {
    lastDisplayTime = millis();
    
    u8g2.clearBuffer();
    if (isMenu) {
      drawMenu();
    } else {
      drawData();
    }
    u8g2.sendBuffer();
  }
}

void drawMenu() {
  u8g2.setDrawColor(1);          
  u8g2.setFont(u8g2_font_5x7_tr); 
  u8g2.setFontPosBaseline();     

  int prevIndex = (menuIndex - 1 + currentMenuLength) % currentMenuLength;
  int nextIndex = (menuIndex + 1) % currentMenuLength;

  u8g2.setCursor(12, 8);  
  u8g2.print(currentMenuPtr[prevIndex].label);
  
  u8g2.setFont(u8g2_font_6x10_tr); 
  u8g2.setCursor(0, 19);  
  u8g2.print("> "); 
  u8g2.print(currentMenuPtr[menuIndex].label);
  
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setCursor(12, 30); 
  u8g2.print(currentMenuPtr[nextIndex].label);
}

void drawData() {
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setDrawColor(1); 

  u8g2.drawBox(0, 0, 128, 9); 
  u8g2.setDrawColor(0); 
  u8g2.setCursor(5, 7);

  double v1, v2, v3;
  const char *l1, *l2, *l3;

  switch(currentState) {
    case STATE_VIEW_ANGLES:
      u8g2.print("ANGLES"); 
      l1="Roll : "; v1=rollZ; l2="Pitch: "; v2=pitchZ; l3="Yaw  : "; v3=yawZ; 
      break;
    case STATE_VIEW_ACCEL:
      u8g2.print("ACCEL");
      l1="Acc X: "; v1=ICM.agmt.acc.axes.x/1000.00; l2="Acc Y: "; v2=ICM.agmt.acc.axes.y/1000.00; l3="Acc Z: "; v3=ICM.agmt.acc.axes.z/1000.00; 
      break;
    case STATE_VIEW_GYRO:
      u8g2.print("GYRO");
      l1="Gyr X: "; v1=ICM.agmt.gyr.axes.x; l2="Gyr Y: "; v2=ICM.agmt.gyr.axes.y; l3="Gyr Z: "; v3=ICM.agmt.gyr.axes.z; 
      break;
    case STATE_VIEW_MAG:
      u8g2.print("MAGNET");
      l1="Mag X: "; v1=ICM.agmt.mag.axes.x; l2="Mag Y: "; v2=ICM.agmt.mag.axes.y; l3="Mag Z: "; v3=ICM.agmt.mag.axes.z; 
      break;
    default:
      u8g2.setDrawColor(1); 
      return;
  }
  
  u8g2.setDrawColor(1); 
  u8g2.setCursor(5, 18); u8g2.print(l1); u8g2.print(v1);
  u8g2.setCursor(5, 25); u8g2.print(l2); u8g2.print(v2);
  u8g2.setCursor(5, 32); u8g2.print(l3); u8g2.print(v3);
}