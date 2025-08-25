// Libraries
#include "picobricksESP32.h"
#include "pitchesESP32.h"
#include <Wire.h>

// Pin Defination
#define BUTTON_PIN 4
#define RELAY_PIN 12
#define BUZZER_PIN 16
#define IR_PIN 17
#define RGB_PIN 32
#define PIR_PIN 33
#define POT_PIN 34
#define LDR_PIN 35

// Defines
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C // I2C address of the OLED display
#define RGB_COUNT 3         // Number of RGB LEDs

// Variables
float temperature;
float humidity;
float potantiometer;
float ldr;
int cs = 0;
int pirState = 0;
volatile int buttonState = 0;
volatile int music_stt = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
char str[10];
int color = 0;
int gesture = 0;
int buttonValue = 0; 
int irCode = 0;
volatile bool irReceived = false;

// Function Declaration
SSD1306 OLED(SCREEN_ADDRESS, SCREEN_WIDTH, SCREEN_HEIGHT);
SHTC3 shtc(0x70);
NeoPixel strip(RGB_COUNT, RGB_PIN);
motorDriver motor;
APDS9960 apds;
CY8CMBR3116 touchSensor;
IRPico ir(IR_PIN);                                           

// Interrupt handler for button press
void buttonInterruptHandler() {
  buttonState = digitalRead(BUTTON_PIN);
}

// Interrupt service routine for IR
void irInterruptHandler() {
  if (ir.decode()) {
    irCode = ir.getCode();
    irReceived = true;
  }
}

void setup() {
  Serial.begin(115200);  // Begin serial communication
  Wire.begin();          // Begin I2C communication
  
  // Pin modes setup
  pinMode(RGB_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(POT_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  
  // Attach interrupt to BUTTON_PIN
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterruptHandler, CHANGE);

  // Attach interrupt to ir
  attachInterrupt(digitalPinToInterrupt(IR_PIN), irInterruptHandler, FALLING);

  // Initialize devices
  OLED.init();
  OLED.clear();
  OLED.drawBitmap(0, 0, Picobricks_img, 128, 64);
  OLED.show();

  shtc.begin();

  strip.begin();
  strip.show();
  strip.setBrightness(50);

  apds.init(COLOR);
  //apds.init(GESTURE);

  touchSensor.init();
  // Turn relay on and off
  digitalWrite(RELAY_PIN, HIGH);
  delay(1000);
  digitalWrite(RELAY_PIN, LOW);
  delay(1000);
}

void loop() {
  OLED.clear();
  if (buttonState == HIGH) {
    for(int i=0; i<50; i++){
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1);
      digitalWrite(BUZZER_PIN, LOW);
      delay(1);
    }
    buttonState = LOW;
  }

  if (irReceived) {
    Serial.println(irCode, HEX);

    if (irCode == number_1) {   // RGB LED Walking light effect
      strip.setPixelColor(0, 0, 0, 0); 
      strip.setPixelColor(1, 0, 0, 0); 
      strip.setPixelColor(2, 0, 0, 0); 
      strip.show(); 
      delay(1000);
      strip.setPixelColor(0, 255, 0, 0); 
      strip.show(); 
      delay(1000);
      strip.setPixelColor(1, 0, 255, 0); 
      strip.show(); 
      delay(1000);
      strip.setPixelColor(2, 0, 0, 255);
      strip.show(); 
      delay(1000); 
    }
    if (irCode == number_2) { // Relay ON
      digitalWrite(RELAY_PIN, HIGH); 
    }
    if (irCode == number_3) { // Relay OFF
      digitalWrite(RELAY_PIN, LOW);
    }
    if (irCode == number_4) { // DC Motors On
      motor.dc(1,255,0);
      motor.dc(2,255,0);
    }
    if (irCode == number_5) { //DC Motors Off
      motor.dc(1,0,0);
      motor.dc(2,0,0);
    }
    if (irCode == number_6) { //Buzzer
      for(int i=0; i<50; i++){
        digitalWrite(BUZZER_PIN, HIGH);
        delay(1);
        digitalWrite(BUZZER_PIN, LOW);
        delay(1);
      }
    }
    if (irCode == number_8) { // Servo1 to 0 degrees
      motor.servo(1,0);
    }
    if (irCode == number_9) { // Servo1 to 90 degrees
      motor.servo(1,90);
    }
    irReceived = false;
    irCode = 0;
  }

  // Check PIR sensor state (motion detection)
  pirState = digitalRead(PIR_PIN);
  if(pirState == 0){  // No motion detected
    strip.setPixelColor(0, 255,0,0);
    strip.setPixelColor(1, 0,255,0);
    strip.setPixelColor(2, 0,0,255);
    strip.show();
  }
  else{  // Motion detected
    strip.setPixelColor(0, 255,0,0);
    strip.setPixelColor(1, 255,0,0);
    strip.setPixelColor(2, 255,0,0);
    strip.show();
  }

  // Read color from APDS9960 sensor
  /*
  color = apds.readColor();
  Serial.println(color);
  delay(200);
  */

  // Read gesture from APDS9960 sensor
  gesture = apds.readGesture();
  if (gesture != 0)
    Serial.println(gesture);

  // Read touch buttons
  /*
  buttonValue = touchSensor.readTouch();
    if (buttonValue == BUTTON_X) {
    //Serial.println("Button X Pressed!");
  }
  else if (buttonValue == BUTTON_Y) {
    //Serial.println("Button Y Pressed!");
  }
  else if (buttonValue == LEFT_BUTTON) {
    //Serial.println("Left Button Pressed!");
  }
  else if (buttonValue == RIGHT_BUTTON) {
    //Serial.println("Right Button Pressed!");
  }
  else if (buttonValue == UP_BUTTON) {
    //Serial.println("Up Button Pressed!");
  }
  else if (buttonValue == DOWN_BUTTON) {
    //Serial.println("Down Button Pressed!");
  }
  else if (buttonValue == BUTTON_C1) {
    //Serial.println("Button C1 Pressed!");
    tone(BUZZER_PIN, NOTE_C4, NOTE_DURATION);
  }
  else if (buttonValue == BUTTON_D) {
    //Serial.println("Button D Pressed!");
    tone(BUZZER_PIN, NOTE_D4, NOTE_DURATION);
  }
  else if (buttonValue == BUTTON_E) {
    //Serial.println("Button E Pressed!");
    tone(BUZZER_PIN, NOTE_E4, NOTE_DURATION); 
  }
  else if (buttonValue == BUTTON_F) {
    //Serial.println("Button F Pressed!");
    tone(BUZZER_PIN, NOTE_F4, NOTE_DURATION);
  }
  else if (buttonValue == BUTTON_G) {
    //Serial.println("Button G Pressed!");
    tone(BUZZER_PIN, NOTE_G4, NOTE_DURATION);
  }
  else if (buttonValue == BUTTON_A) {
    //Serial.println("Button A Pressed!");
    tone(BUZZER_PIN, NOTE_A4, NOTE_DURATION);
  }
  else if (buttonValue == BUTTON_B) {
    //Serial.println("Button B Pressed!");
    tone(BUZZER_PIN, NOTE_B4, NOTE_DURATION);
  }
  else if (buttonValue == BUTTON_C2) {
    //Serial.println("Button C2 Pressed!");
    tone(BUZZER_PIN, NOTE_C5, NOTE_DURATION);
  }
  else {
    noTone(BUZZER_PIN);
  }
  delay(500);
  */

  // Display temperature on OLED
  OLED.setCursor(0, 0);
  OLED.print("Picobricks");
  // Display temperature
  OLED.setCursor(0, 10);
  OLED.print("Temp: ");
  OLED.setCursor(35, 10);
  temperature = shtc.readTemperature();
  Serial.println(temperature);
  sprintf(str, "%.2f", temperature);
  OLED.print(str);
  OLED.setCursor(70, 10);
  OLED.print("C");
  // Display humidity
  OLED.setCursor(0, 20);
  OLED.print("Hum: ");
  OLED.setCursor(35, 20);
  humidity = shtc.readHumidity();
  sprintf(str, "%.2f", humidity);
  OLED.print(str);
  OLED.setCursor(70, 20);
  OLED.print("%");
  // Display potentiometer value
  OLED.setCursor(0, 30);
  OLED.print("Pot: ");
  OLED.setCursor(35, 30);
  potantiometer = (analogRead(POT_PIN) * 3.3) / 4095;
  sprintf(str, "%.2f", potantiometer);
  OLED.print(str);
  OLED.setCursor(70, 30);
  OLED.print("V");
  // Display light sensor value (LDR)
  OLED.setCursor(0, 40);
  OLED.print("Light: ");
  OLED.setCursor(35, 40);
  float ldrRaw = analogRead(LDR_PIN);          // 0..4095
  ldr = ldrRaw * (100.0f / 4095.0f);           // 0 (darkness) → 100 (light)
  ldr = constrain(ldr, 0.0f, 100.0f);
  sprintf(str, "%.2f", ldr);
  OLED.print(str);
  OLED.setCursor(70, 40);
  OLED.print("%");
  //pir state
  OLED.setCursor(0, 50);
  OLED.print("PIR: ");
  OLED.setCursor(35, 50);
  OLED.print(pirState ? "1" : "0");
  OLED.show();  // Update OLED display
}
