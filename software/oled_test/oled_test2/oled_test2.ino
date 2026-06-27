#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ICM_20948.h>

#define SERIAL_PORT Serial

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0


// Initialize the display using the SSD1305 driver (common for 2.23" 128x32 OLEDs).
// U8G2_R0 = No rotation
// U8X8_PIN_NONE = No reset pin (change this to your pin number if you connected RST)
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
ICM_20948_I2C ICM;
int counter = 1;
int buttonPin = 1;
int buttonRead;
int previous = LOW;
bool initialized = false;
unsigned long lastTime = 0;           // the last time the output pin was toggled
unsigned long debounce = 200UL; 


void setup() {
  pinMode(buttonPin, INPUT);
  // Initialize the display
  u8g2.begin();
  
  // Optional: Set I2C clock to 400kHz for faster updates
  Wire.setClock(400000);
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  ICM.begin( WIRE_PORT, AD0_VAL );
  SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
  SERIAL_PORT.println( ICM.statusString() );
  if( ICM.status != ICM_20948_Stat_Ok ){
    SERIAL_PORT.println( "Trying again..." );
    delay(500);
  }
  else{
    initialized = true;
  },
}

void loop() {
  buttonRead = digitalRead(buttonPin);
  if (buttonRead == HIGH && previous == LOW && millis() - lastTime > debounce)
  {
    if (counter < 9){
      counter++;
    }
    else{
      counter = 1;
    }
    lastTime = millis();
  }  
  u8g2.clearBuffer();                  // Clear the internal memory
  if( ICM.dataReady() ){
    ICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    delay(30);
  }else{
    Serial.println("Waiting for data");
    delay(500);
  }
  u8g2.setFont(u8g2_font_ncenR08_tr); 
//  u8g2.drawStr(5, 25, ICM.agmt.tmp.val);   
  u8g2.setCursor(5, 10);
  u8g2.print(counter);
  u8g2.setCursor(5, 21);
  u8g2.print("test");
  u8g2.setCursor(5, 32);
  u8g2.print("test");
  Serial.println(ICM.agmt.tmp.val);
  u8g2.sendBuffer();                   // Transfer internal memory to the display  
//  delay(30);      
}

K