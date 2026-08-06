#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <TimeLib.h>

// Initialize Display
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
ICM_20948_I2C ICM;
SoftwareSerial ss(4, 3);
Adafruit_BNO055 BNO = Adafruit_BNO055(55, 0x29);

// Define Pins
const int rotaryPinA = 34;
const int rotaryPinB = 33;
const int rotaryButtonPin = 37;         // Select
const int backButtonPin = 36;            // Back
const int chipSelect = BUILTIN_SDCARD;  // Teensy SDcard
int potVal = 0;
float steerVal = 0.0;
const int steerMin = -45;
const int steerMax = 45;
const int redPin = 39;
const int greenPin = 35;
const int steerPin = A1;        // GEWIJZIGD: Was A1, is nu A2
const int wheelSpeedPin = 16;   // NIEUW: Analoge Hall sensor op A1
int hallValue = 0;
int hallThreshold = 600;        // Pas deze drempelwaarde aan op basis van je magneetsterkte!
bool magnetDetected = false;
unsigned long lastPulseTime = 0;
float wheelRpm = 0.0;
float speedKmh = 0.0;
const float wheelCircumference = 2.0; // Omtrek van je wiel in meters (Pas dit aan!)

const int maxFiles = 10;
int fileCounter = 0;

String fileList[maxFiles];
String configDir = "/config/";

// --- LOGGING & TIJD VARIABELEN ---
int lastRTCSec = -1;
char cachedDateTime[25];  // Slaat "[YYYY-MM-DD HH:MM:SS" op
String logLine = "";

// --- MEASUREMENT VARIABLES ---
unsigned long measurementStartTime = 0;
String currentFileName = "DATA_001.CSV";

// Define Menu States
enum StateID {
  STATE_MAIN_MENU,
  STATE_SENSOR_MENU,
  STATE_SETTINGS_MENU,
  STATE_MEASURE_MENU,
  STATE_VIEW_ANGLES,
  STATE_VIEW_ACCEL,
  STATE_VIEW_GYRO,
  STATE_VIEW_MAG,
  STATE_ACTION_ZERO_RESET,
  STATE_VIEW_CONFIG,
  STATE_MAINIMU_MENU,
  STATE_VIEW_GPS,
  STATE_VIEW_STEER,
  STATE_VIEW_WHEELSPEED,
  STATE_MEASURING,
  STATE_VIEW_MEASUREMENTS,
  STATE_VIEW_RTC
};

StateID currentState = STATE_MAIN_MENU;

// Menu Structure
struct MenuItem {
  const char* label;
  StateID nextState;
};

const MenuItem mainMenu[] = {
  { "Sensors >", STATE_SENSOR_MENU },
  { "Settings >", STATE_SETTINGS_MENU },
  { "Measure >", STATE_MEASURE_MENU }  
};

// Globale string voor het veilig opslaan van het dynamische label
String mainIMULabel = "Main IMU ()";

MenuItem sensorMenu[] = {
  { "GPS", STATE_VIEW_GPS },
  { mainIMULabel.c_str(), STATE_MAINIMU_MENU },
  { "Steering Angle", STATE_VIEW_STEER },
  { "Wheel Speed", STATE_VIEW_WHEELSPEED }
};

const MenuItem mainIMUMenu[] = { 
  { "Angles", STATE_VIEW_ANGLES }, 
  { "Accel", STATE_VIEW_ACCEL }, 
  { "Gyro", STATE_VIEW_GYRO }, 
  { "Magnet", STATE_VIEW_MAG } 
};

const MenuItem settingsMenu[] = {
  { "Set Zero", STATE_ACTION_ZERO_RESET },
  { "Info", STATE_MAIN_MENU },
  { "Load Config", STATE_VIEW_CONFIG },
  { "View RTC Time", STATE_VIEW_RTC }
};

const MenuItem measureMenu[] = {
  { "Start Measuring", STATE_MEASURING},
  { "Saved Measurements", STATE_VIEW_MEASUREMENTS}
};

MenuItem configMenu[maxFiles];

// Navigation
const MenuItem* currentMenuPtr = mainMenu;
int currentMenuLength = 3;
int menuIndex = 0;

// --- HARDWARE VARIABELEN ---
int aState, aLastState;
int selectBtnState, lastSelectBtnState = HIGH;
int backBtnState, lastBackBtnState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
unsigned long msAtLastRTCSecond = 0;

// config variables
String mainIMU;
bool singleFile = false;

// Display Timer (TEGEN LAG)
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 50;  // Update scherm elke 50ms (20 FPS)

// --- LOGGING TIMER ---
unsigned long lastLogTime = 0;
const unsigned long logInterval = 10; // 50ms = 20Hz (20 metingen per seconde). 

// Sensor Data Variables
double roll = 0.0, pitch = 0.0, yaw = 0.0;
double roll0 = 0.0, pitch0 = 0.0, yaw0 = 0.0;
double rollZ = 0.0, pitchZ = 0.0, yawZ = 0.0;


// --- FUNCTIE DECLARATIES ---
bool menuCheck(StateID state); // Nodig zodat de compiler deze al kent!
void drawMenu();
void drawData();
void applyConfig(String selectedFile);
void listFiles(File dir);
time_t getTeensy3Time();

void setup() {
  Serial.begin(115200);
  setSyncProvider(getTeensy3Time);

  // Korte delay zodat de serial monitor kan openen
  delay(1000);

  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }

  pinMode(rotaryPinA, INPUT_PULLUP);
  pinMode(rotaryPinB, INPUT_PULLUP);
  pinMode(rotaryButtonPin, INPUT_PULLUP);
  pinMode(backButtonPin, INPUT_PULLUP);
  digitalWrite(greenPin, HIGH);

  aLastState = digitalRead(rotaryPinA);

  if (SD.begin(chipSelect)) {
    File folder = SD.open("/config/");
    if (folder) {
      listFiles(folder);
      folder.close();
    } else {
      Serial.println("Kan map /config/ niet vinden!");
    }
  } else {
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
    case STATE_VIEW_RTC:
      currentState = STATE_SETTINGS_MENU;
      switchMenu(settingsMenu, 4);
      break;
    case STATE_VIEW_ANGLES:
    case STATE_VIEW_ACCEL:
    case STATE_VIEW_GYRO:
    case STATE_VIEW_MAG:
      currentState = STATE_MAINIMU_MENU;
      switchMenu(mainIMUMenu, 4); 
      break;
    case STATE_MAINIMU_MENU:
    case STATE_VIEW_GPS:
    case STATE_VIEW_STEER:
    case STATE_VIEW_WHEELSPEED:
      currentState = STATE_SENSOR_MENU;
      switchMenu(sensorMenu, 4); 
      break;
    case STATE_SENSOR_MENU:
    case STATE_SETTINGS_MENU:
    case STATE_MEASURE_MENU:
    case STATE_MEASURING: // Nu kun je met BACK ook de meting verlaten!
      currentState = STATE_MAIN_MENU;
      switchMenu(mainMenu, 3);
      break;
    case STATE_MAIN_MENU:
      break;
  }
}

void handleSelectPress() {
  bool isMenu = menuCheck(currentState);
  if (isMenu) {
    if (currentState == STATE_VIEW_CONFIG) {
      String selectedFile = fileList[menuIndex];
      Serial.print("Loading file: ");
      Serial.println(selectedFile);
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.setCursor(5, 15);
      u8g2.print("Applied:");
      u8g2.setCursor(5, 28);
      u8g2.print(selectedFile);
      u8g2.sendBuffer();
      delay(1000);
      currentState = STATE_SETTINGS_MENU;
      switchMenu(settingsMenu, 4);
      applyConfig(selectedFile);
      return; 
    }

    StateID target = currentMenuPtr[menuIndex].nextState;
    switch (target) {
      case STATE_MAINIMU_MENU:
        currentState = STATE_MAINIMU_MENU;
        switchMenu(mainIMUMenu, 4); 
        break;
      case STATE_SENSOR_MENU:
        currentState = STATE_SENSOR_MENU;
        switchMenu(sensorMenu, 4); 
        break;
      case STATE_SETTINGS_MENU:
        currentState = STATE_SETTINGS_MENU;
        switchMenu(settingsMenu, 4);
        break;
      case STATE_MEASURE_MENU:
        currentState = STATE_MEASURE_MENU;
        switchMenu(measureMenu, 2);
        break;
      case STATE_MEASURING: {
        currentState = STATE_MEASURING;
        measurementStartTime = millis(); // Start de timer
        
        // --- SD BESTANDSNAAM LOGICA ---
        int highestIndex = 0;
        char tempName[20];
        
        // Loop door de SD kaart om te kijken welk nummer we als laatste hebben
        for (int i = 1; i <= 999; i++) {
          // Tip: We gebruiken "DATA001.CSV" om binnen het FAT16/32 8.3 bestandsformaat te blijven.
          snprintf(tempName, sizeof(tempName), "/data/DATA%03d.CSV", i);
          if (SD.exists(tempName)) {
            highestIndex = i;
          } else {
            break; // We hebben de grens gevonden
          }
        }
        
        int fileIndex;
        if (singleFile) {
          // Append modus: pak het laatste bestand, of start bij 1 als de kaart leeg is.
          fileIndex = (highestIndex > 0) ? highestIndex : 1;
        } else {
          // Nieuwe modus: pak het eerstvolgende vrije nummer.
          fileIndex = (highestIndex > 0) ? highestIndex + 1 : 1;
        }
        
        snprintf(tempName, sizeof(tempName), "/data/DATA%03d.CSV", fileIndex);
        currentFileName = String(tempName);
        
        // Open bestand met FILE_WRITE. (Dit append automatisch of creëert een nieuwe)
        File dataFile = SD.open(currentFileName.c_str(), FILE_WRITE);
        if (dataFile) {
          // Als we een *nieuw* bestand hebben aangemaakt (grootte is 0), print een CSV header.
          if (dataFile.size() == 0) {
            dataFile.println("Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,MagX,MagY,MagZ,SteerAngle");
          }
          dataFile.close();
          Serial.print("Klaar voor meting in bestand: ");
          Serial.println(currentFileName);
        } else {
          Serial.println("Fout bij openen/maken van meetbestand!");
          currentFileName = "SD ERROR"; // Laat dit op het schermpje zien
        }
        // ------------------------------
        
        break; 
      }
      case STATE_ACTION_ZERO_RESET:
        roll0 = roll;
        pitch0 = pitch;
        yaw0 = yaw;
        break;
      case STATE_VIEW_CONFIG:
        if (fileCounter > 0) {
          currentState = STATE_VIEW_CONFIG;
          switchMenu(configMenu, fileCounter);
        } else {
          Serial.println("Geen config bestanden gevonden.");
        }
        break;
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

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();

    if (line.length() == 0 || line.startsWith("//") || line.startsWith("#")) {
      continue;
    }

    int equalsIndex = line.indexOf('=');
    if (equalsIndex > 0) {
      String key = line.substring(0, equalsIndex);
      String value = line.substring(equalsIndex + 1);

      key.trim();
      value.trim();

      if (key == "mainIMU") {
        mainIMU = value;
        Serial.print("mainIMU ingeladen: ");
        Serial.println(mainIMU);
        
        mainIMULabel = "Main IMU (" + mainIMU + ")";
        sensorMenu[1].label = mainIMULabel.c_str(); 
      } 
      // --- UPDATE DIT BLOK ---
      else if (key == "singleFile") {
        value.toLowerCase(); // Verander alles naar kleine letters
        // Werkt nu met 'true', '1', en negeert spaties
        singleFile = (value.indexOf("true") >= 0 || value.indexOf("1") >= 0);
        Serial.print("singleFile ingeladen: ");
        Serial.println(singleFile ? "true" : "false");
      }
      // -------------------------
    }
  }
  file.close();
}

void listFiles(File dir) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }
    if (fileCounter < maxFiles) {
      fileList[fileCounter] = String(entry.name());
      configMenu[fileCounter].label = fileList[fileCounter].c_str();
      configMenu[fileCounter].nextState = STATE_VIEW_CONFIG;
      fileCounter++;
    }
    Serial.println(entry.name());
    entry.close();
  }
}

void getICMdata() {
  icm_20948_DMP_data_t data;

  ICM.getAGMT();
  ICM.readDMPdataFromFIFO(&data);

  while ((ICM.status == ICM_20948_Stat_Ok) || (ICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;

      double sumSq = (q1 * q1) + (q2 * q2) + (q3 * q3);
      if (sumSq > 1.0) sumSq = 1.0;
      double q0 = sqrt(1.0 - sumSq);

      double qw = q0;
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      pitch = -atan2(t0, t1) * 180.0 / PI;
      pitchZ = pitch - pitch0;

      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      roll = asin(t2) * 180.0 / PI;
      rollZ = roll - roll0;

      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      yaw = atan2(t3, t4) * 180.0 / PI;
      yawZ = yaw - yaw0;
    }
    ICM.readDMPdataFromFIFO(&data);
  }
}

void getBNOdata() {
  // BNO data functie
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

#define TIME_HEADER "T"
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600;  // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    if (pctime < DEFAULT_TIME) {
      pctime = 0L;
    }
  }
  return pctime;
}

String getFormattedTimestamp() {
  if (timeStatus() == timeSet) {
    int currentSec = second();

    // Als de RTC seconde verspringt, sla dan direct de millis() van dat moment op
    if (currentSec != lastRTCSec) {
      snprintf(cachedDateTime, sizeof(cachedDateTime), "[%04d-%02d-%02d %02d:%02d:%02d",
               year(), month(), day(), hour(), minute(), currentSec);
      lastRTCSec = currentSec;
      
      // Bewaar de starttijd van deze nieuwe seconde
      msAtLastRTCSecond = millis(); 
    }

    // Bereken het aantal milliseconden verstreken sinds de RTC seconde begon
    unsigned long currentMs = millis() - msAtLastRTCSecond;
    
    // Kleine klokafwijkingen kunnen currentMs iets over de 1000 duwen voordat de 
    // RTC zijn update krijgt. Zorg dat het max 999 blijft voor nette CSV formattering.
    if (currentMs > 999) {
      currentMs = 999;
    }

    char fullTimestamp[35];
    snprintf(fullTimestamp, sizeof(fullTimestamp), "%s.%03d]", cachedDateTime, currentMs);

    return String(fullTimestamp);
  } else {
    char fallback[20];
    snprintf(fallback, sizeof(fallback), "[ms: %lu]", millis());
    return String(fallback);
  }
}

void loop() {
  getICMdata();
  potVal = analogRead(steerPin);
  steerVal = map(potVal, 0, 1023, steerMin * 100, steerMax * 100) / 100.0;
  hallValue = analogRead(wheelSpeedPin);
  if (hallValue > hallThreshold && !magnetDetected) {
    magnetDetected = true;
    unsigned long currentTime = millis();
    unsigned long timeDiff = currentTime - lastPulseTime;
    if (timeDiff > 10) { 
      wheelRpm = 60000.0 / timeDiff; // 1 minuut = 60000 ms
      speedKmh = (wheelRpm * 60.0 * wheelCircumference) / 1000.0;
    }
    lastPulseTime = currentTime;
  } else if (hallValue < (hallThreshold - 50)) { 
    magnetDetected = false; // Magneet is weer weg
  }
  if (millis() - lastPulseTime > 2000) {
    wheelRpm = 0.0;
    speedKmh = 0.0;
  }
  Serial.println(hallValue);
  Serial.println(speedKmh);
  while (ss.available() > 0) {
    byte gpsData = ss.read();
    Serial.write(gpsData);
  }

  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t);
      setTime(t);
    }
  }


  logLine = getFormattedTimestamp();
  // Serial.println(logLine);

  if (currentState == STATE_MEASURING) {
    saveDataToFile();
  }

  bool isMenu = menuCheck(currentState);

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

  // --- KNOPPEN ---
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
    if (digitalRead(backButtonPin) == LOW) handleBackPress();
  }
  lastBackBtnState = readingBack;

  // --- DISPLAY TEKENEN (Met Timer!) ---
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

void saveDataToFile() {
  // Controleer of het tijd is om een nieuwe meting op te slaan
  if (millis() - lastLogTime >= logInterval) {
    lastLogTime = millis();

    // Open het bestand in append-modus
    File dataFile = SD.open(currentFileName.c_str(), FILE_WRITE);
    
    if (dataFile) {
      // Direct wegschrijven (Véél stabieler dan String concatenatie)
      dataFile.print(logLine); dataFile.print(",");
      
      dataFile.print(rollZ, 2); dataFile.print(",");
      dataFile.print(pitchZ, 2); dataFile.print(",");
      dataFile.print(yawZ, 2); dataFile.print(",");
      
      dataFile.print(ICM.agmt.acc.axes.x / 1000.00, 3); dataFile.print(",");
      dataFile.print(ICM.agmt.acc.axes.y / 1000.00, 3); dataFile.print(",");
      dataFile.print(ICM.agmt.acc.axes.z / 1000.00, 3); dataFile.print(",");
      
      dataFile.print(ICM.agmt.gyr.axes.x); dataFile.print(",");
      dataFile.print(ICM.agmt.gyr.axes.y); dataFile.print(",");
      dataFile.print(ICM.agmt.gyr.axes.z); dataFile.print(",");
      
      dataFile.print(ICM.agmt.mag.axes.x); dataFile.print(",");
      dataFile.print(ICM.agmt.mag.axes.y); dataFile.print(",");
      dataFile.print(ICM.agmt.mag.axes.z); dataFile.print(",");
      
      // Let op: de laatste gebruikt println voor de "Enter"
      dataFile.println(steerVal, 2); 

      // Sluit af om data veilig op te slaan
      dataFile.close();
    } else {
      Serial.println("Fout bij schrijven van data naar SD!");
    }
  }
}

void drawData() {
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setDrawColor(1);

  u8g2.drawBox(0, 0, 128, 9);
  u8g2.setDrawColor(0);
  u8g2.setCursor(5, 7);

  double v1 = 0, v2 = 0, v3 = 0; // Geïnitialiseerd om warnings te voorkomen
  const char *l1 = "", *l2 = "", *l3 = "";

  switch (currentState) {
    case STATE_VIEW_RTC: {
      u8g2.print("RTC TIME");
      
      // Controleer of de tijd is gesynchroniseerd
      if (timeStatus() == timeSet) {
        char dateStr[20];
        char timeStr[20];
        
        // Formatteer naar YYYY-MM-DD en HH:MM:SS
        snprintf(dateStr, sizeof(dateStr), "%04d-%02d-%02d", year(), month(), day());
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hour(), minute(), second());

        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setDrawColor(1);
        
        u8g2.setCursor(5, 18);
        u8g2.print("Date: ");
        u8g2.print(dateStr);
        
        u8g2.setCursor(5, 28);
        u8g2.print("Time: ");
        u8g2.print(timeStr);
      } else {
        // Als de RTC nog geen tijd heeft ontvangen
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.setDrawColor(1);
        u8g2.setCursor(5, 18);
        u8g2.print("RTC Not Synced!");
        u8g2.setCursor(5, 28);
        u8g2.print("Waiting for GPS/PC...");
      }
      return; // Stop hier, zodat we de v1/v2/v3 printlogica overslaan
    }
    case STATE_MEASURING: {
      u8g2.print("MEASURING");
      
      // Bereken verstreken tijd in seconden en minuten
      unsigned long elapsedMillis = millis() - measurementStartTime;
      unsigned long elapsedSeconds = elapsedMillis / 1000;
      int mins = elapsedSeconds / 60;
      int secs = elapsedSeconds % 60;
      
      // Formatteer de tijd als MM:SS
      char timeStr[10];
      snprintf(timeStr, sizeof(timeStr), "%02d:%02d", mins, secs);

      // Teken filenaam
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.setDrawColor(1);
      u8g2.setCursor(5, 18);
      u8g2.print("File: ");
      u8g2.print(currentFileName);
      
      // Teken timer
      u8g2.setCursor(5, 28);
      u8g2.print("Time: ");
      u8g2.print(timeStr);
      
      return; // Stop hier, de v1/v2/v3 logica hieronder is niet nodig voor de timer
    }
    case STATE_VIEW_ANGLES:
      u8g2.print("ANGLES");
      l1 = "Roll : ";
      v1 = rollZ;
      l2 = "Pitch: ";
      v2 = pitchZ;
      l3 = "Yaw  : ";
      v3 = yawZ;
      break;
    case STATE_VIEW_ACCEL:
      u8g2.print("ACCEL");
      l1 = "Acc X: ";
      v1 = ICM.agmt.acc.axes.x / 1000.00;
      l2 = "Acc Y: ";
      v2 = ICM.agmt.acc.axes.y / 1000.00;
      l3 = "Acc Z: ";
      v3 = ICM.agmt.acc.axes.z / 1000.00;
      break;
    case STATE_VIEW_GYRO:
      u8g2.print("GYRO");
      l1 = "Gyr X: ";
      v1 = ICM.agmt.gyr.axes.x;
      l2 = "Gyr Y: ";
      v2 = ICM.agmt.gyr.axes.y;
      l3 = "Gyr Z: ";
      v3 = ICM.agmt.gyr.axes.z;
      break;
    case STATE_VIEW_MAG:
      u8g2.print("MAGNET");
      l1 = "Mag X: "; v1 = ICM.agmt.mag.axes.x;
      l2 = "Mag Y: "; v2 = ICM.agmt.mag.axes.y;
      l3 = "Mag Z: "; v3 = ICM.agmt.mag.axes.z;
      break;
    case STATE_VIEW_STEER:
      u8g2.print("STEER");
      l1 = "Raw Value: "; v1 = potVal;
      l2 = "Angle: "; v2 = steerVal;
      l3 = ""; v3 = 0;
      break;
    case STATE_VIEW_WHEELSPEED:
      u8g2.print("WHEEL SPEED");
      l1 = "Raw: "; v1 = hallValue;      // Ruwe sensorwaarde (handig voor het kalibreren van je threshold)
      l2 = "RPM: "; v2 = wheelRpm;       // Berekende toeren per minuut
      l3 = "Km/h: "; v3 = speedKmh;      // Berekende snelheid in km/u
      break;
    default:
      u8g2.setDrawColor(1);
      return;
  }
  
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setDrawColor(1);
  u8g2.setCursor(5, 18);
  u8g2.print(l1);
  u8g2.print(v1);
  u8g2.setCursor(5, 25);
  u8g2.print(l2);
  u8g2.print(v2);
  u8g2.setCursor(5, 32);
  u8g2.print(l3);
  if (l3[0] != '\0') u8g2.print(v3); // Print v3 alleen als de tekst in l3 niet leeg is
}

bool menuCheck(StateID state) {
  bool isMenu = false;
  switch (state) {
    case STATE_MAIN_MENU:
    case STATE_SENSOR_MENU:
    case STATE_SETTINGS_MENU:
    case STATE_VIEW_CONFIG:
    case STATE_MAINIMU_MENU:
    case STATE_MEASURE_MENU:
      isMenu = true;
      break; // Netjes afsluiten
    default:
      isMenu = false;
      break;
  }
  return isMenu;
}