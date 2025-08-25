#include <Wire.h>

//------------------IR--------------------
#define number_1 0xA2
#define number_2 0x62
#define number_3 0xE2
#define number_4 0x22
#define number_5 0x02
#define number_6 0xC2
#define number_7 0xE0
#define number_8 0xA8
#define number_9 0x90
#define number_0 0x98
#define button_up 0x18
#define button_down 0x4A
#define button_right 0x5A
#define button_left 0x10
#define button_ok 0x38
#define button_star 0x68
#define button_sharp 0xB0

class SSD1306 {
public:
    SSD1306(uint8_t address, uint8_t width, uint8_t height);
    void init();    // Initialize display
    void clear();   // Clear the screen
    void setCursor(uint8_t x, uint8_t y);   // Set text cursor position
    void print(const char* text);   // Print text on the screen
    void show();    // Send buffer to display
    void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint8_t w, uint8_t h); // Draw image

private:
    uint8_t _address;   // I2C address of the display
    uint8_t _width;     // Display dimensions
    uint8_t _height;    // Display dimensions
    uint8_t _cursorX;   // Cursor position
    uint8_t _cursorY;   // Cursor position
    uint8_t _buffer[1024];  // Display memory buffer

    void sendCommand(uint8_t command);  // Send command to OLED
    void sendData(uint8_t data);    // Send data to OLED
    void drawChar(int16_t x, int16_t y, unsigned char c);   // Draw single character
    void drawPixel(uint8_t x, uint8_t y, bool color);   // Draw a pixel
};

/******************************
 Ir Reciever
*******************************/
class IRPico {
public:
    IRPico(uint8_t recv_pin);
    bool decode();
    uint32_t getCode();
private:
    uint8_t recvPin;
    uint32_t receivedCode;
    bool receivedFlag;
    uint32_t waitForState(bool state, uint32_t timeout_us);
};

class SHTC3 {
public:
    SHTC3(uint8_t address = 0x70); // Constructor to initialize the I2C address (default: 0x70)
    void begin();                    // Initializes the sensor
    float readTemperature();          // Reads and returns the temperature in Celsius
    float readHumidity();             // Reads and returns the humidity in percentage
    void wakeUp();                    // Wakes up the sensor from sleep mode
    void sleep();                     // Puts the sensor into sleep mode

private:
    uint8_t _address;                 // I2C address for the SHTC3 sensor
    void sendCommand(uint8_t high, uint8_t low); // Sends a command to the sensor
    void readData(uint8_t *data, uint8_t len);   // Reads data from the sensor
};

void espShow(uint8_t pin, uint8_t *pixels, uint32_t numBytes);

class NeoPixel {
public:
  NeoPixel(uint16_t n, int16_t p);

  void begin(void);
  void show(void);
  void setPin(int16_t p);
  void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
  void setBrightness(uint8_t);
  void updateLength(uint16_t n);
  void updateType();
  bool canShow(void) {
    uint32_t now = micros();
    if (endTime > now) {
      endTime = now;
    }
    return (now - endTime) >= 300L;
  }
  
  uint8_t *getPixels(void) const { return pixels; };
  uint8_t getBrightness(void) const;

protected:
  bool begun;         ///< true if begin() previously called
  uint16_t numLEDs;   ///< Number of RGB LEDs in strip
  uint16_t numBytes;  ///< Size of 'pixels' buffer below
  int16_t pin;        ///< Output pin number (-1 if not yet set)
  uint8_t brightness; ///< Strip brightness 0-255 (stored as +1)
  uint8_t *pixels;    ///< Holds LED color values (3 or 4 bytes each)
  uint8_t rOffset;    ///< Red index within each 3- or 4-byte pixel
  uint8_t gOffset;    ///< Index of green byte
  uint8_t bOffset;    ///< Index of blue byte
  uint8_t wOffset;    ///< Index of white (==rOffset if no white)
  uint32_t endTime;   ///< Latch timing reference
};

class motorDriver {
public:
    motorDriver();
    void dc(int dcNumber, int speed, int direction);
    void servo(int servoNumber, int angle);
};

#define APDS_ADDRESS 0x39
#define APDS9960_THRESHOLD_OUT 10
#define APDS9960_SENSITIVITY_1 50

#define COLOR   1
#define GESTURE 2

#define NONE  0
#define RED   1
#define GREEN 2
#define BLUE  3

#define UP    1
#define DOWN  2
#define LEFT  3
#define RIGHT 4

class APDS9960 {
public:
    APDS9960();
    void init(uint8_t type);
    int readColor();
    int readGesture();

private:
    uint8_t control;
    uint8_t control2;
    uint8_t red_L;
    uint8_t red_H;
    uint8_t green_L;
    uint8_t green_H;
    uint8_t blue_L;
    uint8_t blue_H;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint8_t gstatus;
    uint8_t fifo_level;
    uint8_t data;
    uint8_t u_data[32];
    uint8_t d_data[32];
    uint8_t l_data[32];
    uint8_t r_data[32];
};

/* CY8CMBR3116 Register Map Offset Address */
#define CHIP_ADDRESS 0x37
#define PROX_STAT 0xAE
#define BUTTON_STATUS 0xAA
#define NOTE_DURATION 200

// Define button mappings
#define BUTTON_X 1
#define BUTTON_Y 2
#define LEFT_BUTTON 3
#define RIGHT_BUTTON 4
#define UP_BUTTON 5
#define DOWN_BUTTON 6
#define BUTTON_C1 7
#define BUTTON_D 8
#define BUTTON_E 9
#define BUTTON_F 10
#define BUTTON_G 11
#define BUTTON_A 12
#define BUTTON_B 13
#define BUTTON_C2 14

class CY8CMBR3116 {
public:
    CY8CMBR3116();
    void init();
    int readTouch();
    
private:
    uint8_t val[1];       // Variable to store proximity data
    uint8_t buff[2];      // Variable to store button states
    int proximityCounter; // Counter for proximity sensor
    int proximityStatus;  // Status of proximity sensor
    int value;
};
