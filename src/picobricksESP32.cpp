#include "picobricksESP32.h"
#include "fontESP32.h"

/*****************************
SSD1306 LIBRARY
******************************/

// Constructor to initialize the SSD1306 display
SSD1306::SSD1306(uint8_t address, uint8_t width, uint8_t height)
    : _address(address), _width(width), _height(height), _cursorX(0), _cursorY(0) {
    memset(_buffer, 0, sizeof(_buffer));  // Initialize the buffer to store the pixel data
}

// Function to initialize the SSD1306 display with necessary commands
void SSD1306::init() {
    //Wire.begin();
    sendCommand(0xAE); // Display OFF
    sendCommand(0xD5); // Set Display Clock
    sendCommand(0x80); // Suggested value
    sendCommand(0xA8); // Set Multiplex Ratio
    sendCommand(_height - 1); // Set height
    sendCommand(0xD3); // Set Display Offset
    sendCommand(0x00); // No offset
    sendCommand(0x40); // Set Display Start Line
    sendCommand(0x8D); // Charge Pump
    sendCommand(0x14); // Enable Charge Pump
    sendCommand(0x20); // Memory Addressing Mode
    sendCommand(0x00); // Horizontal Addressing
    sendCommand(0xA1); // Set Segment Re-map
    sendCommand(0xC8); // Set COM Output Scan Direction
    sendCommand(0xDA); // Set COM Pins Hardware
    sendCommand(0x12); // Select hardware configuration
    sendCommand(0x81); // Set Contrast
    sendCommand(0xCF); // Max contrast
    sendCommand(0xD9); // Set Pre-charge Period
    sendCommand(0xF1); // Pre-charge value
    sendCommand(0xDB); // Set VCOMH Deselect Level
    sendCommand(0x40); // Set VCOMH level
    sendCommand(0xA4); // Entire Display ON
    sendCommand(0xA6); // Set Normal/Inverse Display
    sendCommand(0xAF); // Display ON
}

// Clear the display and reset the cursor position
void SSD1306::clear() {
    memset(_buffer, 0, sizeof(_buffer)); // Reset the buffer
    _cursorX = 0; // Reset the cursor to the starting position
    _cursorY = 0; // Reset the cursor to the top
}

// Set the cursor position for drawing text or characters
void SSD1306::setCursor(uint8_t x, uint8_t y) {
    _cursorX = x; // Set the X position
    _cursorY = y; // Set the Y position
}

// Function to print a string on the display starting from the cursor position
void SSD1306::print(const char* text) {
    while (*text) {
        drawChar(_cursorX, _cursorY, *text++); // Draw each character
        _cursorX += 6; // Move the cursor to the right (space for the next character)
        if (_cursorX + 6 >= _width) { // If the cursor reaches the end of the screen
            _cursorX = 0; // Reset X to 0 (new line)
            _cursorY += 8;  // Move the cursor down (new line)
        }
    }
}

// Display the buffer content on the screen
void SSD1306::show() {
    for (uint8_t i = 0; i < 8; i++) {
        sendCommand(0xB0 + i); // Set the page start address
        sendCommand(0x00); // Low column start address
        sendCommand(0x10); // High column start address
        for (uint8_t j = 0; j < 128; j++) {
            sendData(_buffer[i * 128 + j]);
        }
    }
}

// Send a command to the OLED
void SSD1306::sendCommand(uint8_t command) {
    Wire.beginTransmission(_address); // Start communication with the display
    Wire.write(0x00); // Command mode
    Wire.write(command); // Send the command
    Wire.endTransmission(); // End communication
}

// Send data to the OLED (for pixels)
void SSD1306::sendData(uint8_t data) {
    Wire.beginTransmission(_address); // Start communication with the display
    Wire.write(0x40); // Data mode
    Wire.write(data); // Send the data
    Wire.endTransmission(); // End communication
}

// Draw a character at the given position (x, y)
void SSD1306::drawChar(int16_t x, int16_t y, unsigned char c) {
    if (c >= '0' && c <= '9') { // If the character is a number
        c = c - '0'; 
    } 
    else if (c >= 'A' && c <= 'Z') {  // If the character is an uppercase letter
        c = c - 'A' + 10; 
    } 
    else if (c >= 'a' && c <= 'z') {  // If the character is a lowercase letter
        c = c - 'a' + 36; 
    }
    else if (c >= '!' && c <= '/') {  // Special characters
        c = c - '!' + 62; 
    }
    else if (c >= ':' && c <= '@') {  // Special characters 2
        c = c - ':' + 77; 
    }
    else {
        return; // If the character is not supported, do nothing
    }

    for (int i = 0; i < 5; i++) {
        uint8_t line = font[c][i];  // Get the pixel data for the character
        for (int j = 0; j < 8; j++) {
            if (line & (1 << j)) { // If the pixel is set (bit is 1)
                drawPixel(x + i, y + j, true);  // Draw a filled pixel
            } else {
                drawPixel(x + i, y + j, false);  // Draw an empty pixel
            }
        }
    }
}

// Draw a pixel at the given (x, y) position with the specified color (true for ON, false for OFF)
void SSD1306::drawPixel(uint8_t x, uint8_t y, bool color) {
    if (x >= _width || y >= _height) return; // If the pixel is out of bounds, do nothing
    if (color) {
        _buffer[x + (y / 8) * _width] |= (1 << (y % 8)); // Set the bit for ON
    } else {
        _buffer[x + (y / 8) * _width] &= ~(1 << (y % 8)); // Clear the bit for OFF
    }
}

void SSD1306::drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint8_t w, uint8_t h) {
  uint16_t byteWidth = (w + 7) / 8;
  for (uint8_t j = 0; j < h; j++) {
    for (uint8_t i = 0; i < w; i++) {
      uint8_t byte = bitmap[j * byteWidth + i / 8];
      if (byte & (128 >> (i & 7))) {
        drawPixel(x + i, y + j, true);
      }
    }
  }
}

/*****************************
IR LIBRARY
******************************/
// Constructor: Initialize the IR receiver pin
IRPico::IRPico(uint8_t recv_pin) {
    recvPin = recv_pin;
    pinMode(recvPin, INPUT_PULLUP); // Set pin as input with internal pull-up resistor
    receivedCode = 0;
    receivedFlag = false;
}

// Wait for the specified pin state (HIGH or LOW) within a given timeout (microseconds)
uint32_t IRPico::waitForState(bool state, uint32_t timeout_us) {
    uint32_t start = micros();
    while (digitalRead(recvPin) != state) {
        if ((micros() - start) > timeout_us) {
            return 0; // Timeout occurred
        }
    }
    return micros() - start; // Return how long it took
}

// Decode an incoming NEC signal
bool IRPico::decode() {
    // Wait for the start signal (should be LOW)
    if (waitForState(LOW, 10000) == 0) return false;

    // Wait for 9ms LOW (start burst)
    uint32_t lowTime = waitForState(HIGH, 15000);
    if (lowTime < 8000 || lowTime > 10000) {
        return false; // Incorrect start LOW timing
    }

    // Wait for 4.5ms HIGH (space after start)
    uint32_t highTime = waitForState(LOW, 8000);
    if (highTime < 4000 || highTime > 5000) {
        return false; // Incorrect start HIGH timing
    }

    // Start receiving 32 bits of data
    receivedCode = 0;
    for (int i = 0; i < 32; i++) {
        // Wait for LOW (560us expected)
        if (waitForState(HIGH, 1000) == 0) return false;

        // Measure the length of HIGH signal
        uint32_t t = waitForState(LOW, 3000);
        if (t == 0) return false;

        if (t > 1000) {
            // If HIGH is longer than 1ms, it's a logical '1'
            receivedCode = (receivedCode << 1) | 1;
        } else {
            // Otherwise, it's a logical '0'
            receivedCode = (receivedCode << 1);
        }
    }

    receivedFlag = true; // Mark that a valid code was received
    return true;
}

// Get the received command (extracts the 8-bit command from the 32-bit NEC frame)
uint32_t IRPico::getCode() {
    if (receivedFlag) {
        receivedFlag = false; // Clear the flag
        return (receivedCode >> 8) & 0xFF; // Extract and return the command byte
    }
    return 0; // No new code received
}

/*****************************
SHTC3 LIBRARY
******************************/

SHTC3::SHTC3(uint8_t address) {
    _address = address;
}

/**
 * Initializes the SHTC3 sensor by starting the I2C communication
 * and waking up the sensor to start measurements.
 */
void SHTC3::begin() {
    Wire.begin(); // Start the I2C bus
    wakeUp();     // Wake up the sensor to begin measuring
}

/**
 * Sends a command to the SHTC3 sensor.
 * @param high The high byte of the command.
 * @param low The low byte of the command.
 */
void SHTC3::sendCommand(uint8_t high, uint8_t low) {
    Wire.beginTransmission(_address); // Begin communication with the sensor
    Wire.write(high); // Send the high byte of the command
    Wire.write(low); // Send the low byte of the command
    Wire.endTransmission(); // End the communication
}

/**
 * Reads data from the SHTC3 sensor.
 * @param data Pointer to the buffer where the data will be stored.
 * @param len The number of bytes to read.
 */
void SHTC3::readData(uint8_t *data, uint8_t len) {
    Wire.requestFrom(_address, len); // Request 'len' bytes of data from the sensor
    uint8_t index = 0;
    while (Wire.available() && index < len) {
        data[index++] = Wire.read(); // Store the data in the buffer
    }
}

/**
 * Wakes up the SHTC3 sensor to begin measurement.
 * Sends the appropriate wake-up command and waits for the sensor to be ready.
 */
void SHTC3::wakeUp() {
    sendCommand(0x35, 0x17); // Wake up command (start measurement)
    delay(500);               // Delay to allow sensor to wake up
}

/**
 * Puts the SHTC3 sensor to sleep to conserve power.
 * Sends the sleep command to turn off the sensor's measurement functions.
 */
void SHTC3::sleep() {
    sendCommand(0xB0, 0x64); // Sleep command
}

/**
 * Reads the temperature value from the SHTC3 sensor.
 * The temperature is in Celsius and is calculated based on the sensor's raw data.
 * @return The temperature in Celsius.
 */
float SHTC3::readTemperature() {
    uint8_t data[6];
    sendCommand(0x78, 0x66); // Start measurement for temperature
    delay(100);               // Wait for the measurement
    readData(data, 6);        // Read the 6 bytes of data (2 for temperature, 2 for humidity)
    
    uint16_t rawTemp = (data[0] << 8) | data[1]; // Combine the 2 bytes
    rawTemp = rawTemp & 0xFFFC;  // Mask the last two bits (for the CRC check)
    return (((4375 * rawTemp) >> 14) - 4500) / 100.0;  // Convert raw data to temperature in Celsius based on sensor's formula
}

/**
 * Reads the humidity value from the SHTC3 sensor.
 * The humidity is in percentage and is calculated based on the sensor's raw data.
 * @return The humidity in percentage.
 */
float SHTC3::readHumidity() {
    uint8_t data[6];
    sendCommand(0x78, 0x66); // Start measurement for humidity
    delay(100);               // Wait for the measurement
    readData(data, 6);        // Read the 6 bytes of data (2 for temperature, 2 for humidity)

    uint16_t rawHumidity = (data[3] << 8) | data[4]; // Combine the 2 bytes for humidity
    rawHumidity = rawHumidity & 0xFFFC; // Mask the last two bits (for the CRC check)
    return ((625 * rawHumidity) >> 12) / 100.0; // Convert raw humidity data to percentage based on sensor's formula
}

/*****************************
NEOPIXEL LIBRARY
******************************/

// Constructor for NeoPixel class
// Initializes the NeoPixel object with the number of LEDs (n) and pin (p).
// It also sets initial values for other parameters.
NeoPixel::NeoPixel(uint16_t ledCount, int pin
#ifdef PB_CORE_V2
                   , rmt_channel_t ch
#endif
)
: _pin(pin),
  _pin_enum((gpio_num_t)pin),
  _ledCount(ledCount),
  _brightness(255),
  _pixels(nullptr),
  _begun(false)
#ifdef PB_CORE_V2
 , _channel(ch)
 , _items(nullptr)
 , _itemsLen(ledCount * 24)
#endif
#ifdef PB_CORE_V3
 , _txChannel(nullptr)
 , _ledEncoder(nullptr)
 , _txConfig{}
#endif
{}

NeoPixel::~NeoPixel() { end(); }

void NeoPixel::applyBrightness(uint8_t &r, uint8_t &g, uint8_t &b) const {
  if (_brightness == 255) return;
  uint16_t br = _brightness + 1; // 1–256
  r = (uint8_t)((r * br) >> 8);
  g = (uint8_t)((g * br) >> 8);
  b = (uint8_t)((b * br) >> 8);
}

void NeoPixel::clear() { if (_pixels) memset(_pixels, 0, _ledCount * 3); }

void NeoPixel::setBrightness(uint8_t b) { _brightness = b; }

void NeoPixel::setPixelColor(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
  if (i >= _ledCount || !_pixels) return;
  applyBrightness(r, g, b);
  uint16_t base = i * 3;
  _pixels[base + 0] = g; // GRB
  _pixels[base + 1] = r;
  _pixels[base + 2] = b;
}

void NeoPixel::fill(uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < _ledCount; ++i) setPixelColor(i, r, g, b);
}

// ---------- begin() ----------
bool NeoPixel::begin() {
  if (_begun) return true;

  // Pixel buffer
  _pixels = (uint8_t*)malloc(_ledCount * 3);
  if (!_pixels) { end(); return false; }
  clear();

#ifdef PB_CORE_V2
  // ---- Legacy RMT settings ----
  rmt_config_t config = {};
  config.rmt_mode = RMT_MODE_TX;
  config.channel = _channel;
  config.gpio_num = _pin_enum;
  config.mem_block_num = 1;
  config.clk_div = RMT_CLK_DIV;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  if (rmt_config(&config) != ESP_OK) { end(); return false; }
  if (rmt_driver_install(_channel, 0, 0) != ESP_OK) { end(); return false; }

  _itemsLen = _ledCount * 24;
  _items = (rmt_item32_t*)malloc(_itemsLen * sizeof(rmt_item32_t));
  if (!_items) { end(); return false; }
#endif

#ifdef PB_CORE_V3
  // ---- New RMT: TX channel + BYTES encoder ----
  rmt_tx_channel_config_t tx_cfg = {};
  tx_cfg.gpio_num = _pin_enum;
  tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_cfg.resolution_hz = RMT_RESOLUTION_HZ; // 0.1us/step
  tx_cfg.mem_block_symbols = 64;   
  tx_cfg.trans_queue_depth = 4;

  if (rmt_new_tx_channel(&tx_cfg, &_txChannel) != ESP_OK) { end(); return false; }
  // T0H ~0.4us => 4, T0L ~0.85us => 9
  // T1H ~0.8us => 8, T1L ~0.45us => 5
  rmt_bytes_encoder_config_t enc_cfg = {};
  enc_cfg.bit0.duration0 = 4;  enc_cfg.bit0.level0 = 1;
  enc_cfg.bit0.duration1 = 9;  enc_cfg.bit0.level1 = 0;
  enc_cfg.bit1.duration0 = 8;  enc_cfg.bit1.level0 = 1;
  enc_cfg.bit1.duration1 = 5;  enc_cfg.bit1.level1 = 0;
  enc_cfg.flags.msb_first = 1; // WS2812 MSB-first

  if (rmt_new_bytes_encoder(&enc_cfg, &_ledEncoder) != ESP_OK) { end(); return false; }

  if (rmt_enable(_txChannel) != ESP_OK) { end(); return false; }

  memset(&_txConfig, 0, sizeof(_txConfig));
  _txConfig.loop_count = 0; 
#endif

  _begun = true;
  return true;
}

// ---------- end() ----------
void NeoPixel::end() {
#ifdef PB_CORE_V2
  if (_items) { free(_items); _items = nullptr; }
  if (_begun) rmt_driver_uninstall(_channel);
#endif
#ifdef PB_CORE_V3
  if (_begun && _txChannel) { rmt_disable(_txChannel); }
  if (_ledEncoder) { rmt_del_encoder(_ledEncoder); _ledEncoder = nullptr; }
  if (_txChannel)  { rmt_del_channel(_txChannel); _txChannel = nullptr; }
#endif
  if (_pixels) { free(_pixels); _pixels = nullptr; }
  _begun = false;
}

#ifdef PB_CORE_V2
// ---------- Legacy: pixel -> RMT item ----------
void NeoPixel::buildItems() {
  size_t idx = 0;
  for (uint16_t led = 0; led < _ledCount; ++led) {
    const uint8_t g = _pixels[led*3 + 0];
    const uint8_t r = _pixels[led*3 + 1];
    const uint8_t b = _pixels[led*3 + 2];
    const uint8_t grb[3] = { g, r, b };
    for (int c = 0; c < 3; ++c) {
      for (int bit = 7; bit >= 0; --bit) {
        const bool one = (grb[c] >> bit) & 0x01;
        _items[idx].duration0 = one ? T1H_TICKS : T0H_TICKS;
        _items[idx].level0    = 1;
        _items[idx].duration1 = one ? T1L_TICKS : T0L_TICKS;
        _items[idx].level1    = 0;
        idx++;
      }
    }
  }
}
#endif

// ---------- show() ----------
void NeoPixel::show() {
  if (!_begun || !_pixels) return;

#ifdef PB_CORE_V2
  buildItems();
  rmt_write_items(_channel, _items, _itemsLen, true);
  rmt_wait_tx_done(_channel, pdMS_TO_TICKS(10));
  delayMicroseconds(80);
#endif

#ifdef PB_CORE_V3
  size_t nbytes = (size_t)_ledCount * 3;
  if (rmt_transmit(_txChannel, _ledEncoder, _pixels, nbytes, &_txConfig) == ESP_OK) {
    rmt_tx_wait_all_done(_txChannel, pdMS_TO_TICKS(20));
  }
  delayMicroseconds(80); // Treset
#endif
}


/*****************************
MOTOR DRIVER LIBRARY
******************************/
motorDriver::motorDriver(){}

// Function to control a DC motor (1 or 2) with a given speed (0-255) and direction (0 or 1)
// dcNumber: 1 or 2 (selects motor)
// speed: 0 to 255 (controls the motor speed)
// direction: 0 for one direction, 1 for the opposite direction
void motorDriver::dc(int dcNumber, int speed, int direction) {  
  Wire.begin(); // Start the I2C communication
  Wire.beginTransmission(0x22); 
  Wire.write(0x26);      
  Wire.write(dcNumber);   
  Wire.write(speed);  
  Wire.write(direction);
  int cs = dcNumber ^ speed ^ direction;
  Wire.write(cs);
  Wire.endTransmission(); 
}

// Function to control a servo motor (1 to 4) with a given angle (0-180)
// servoNumber: 1, 2, 3, or 4 (selects servo motor)
// angle: 0 to 180 (controls the servo angle)
void motorDriver::servo(int servoNumber, int angle) { 
  Wire.begin(); // Start the I2C communication
  Wire.beginTransmission(0x22); 
  Wire.write(0x26);        
  Wire.write(servoNumber + 2);
  Wire.write(0x00);
  Wire.write(angle);
  int cs = ((servoNumber + 2) ^ angle);
  Wire.write(cs);
  Wire.endTransmission(); 
}

/*****************************
APDS9960 LIBRARY
******************************/
APDS9960::APDS9960(){}

// The init function initializes the APDS9960 sensor and configures it based on the type parameter.
// It sends several I2C commands to set up the sensor for either color or gesture detection.
// If the 'type' parameter is COLOR, it configures the sensor for color detection.
// If the 'type' parameter is GESTURE, it sets up the sensor for gesture detection.
void APDS9960::init(uint8_t type) {
  Wire.begin();  // Start I2C communication
  Wire.beginTransmission(APDS_ADDRESS);
  Wire.write(0x81);  //APDS9960_ATIME
  Wire.write(0xFC);  //APDS9960_GFIFO_U
  Wire.endTransmission();
  Wire.beginTransmission(APDS_ADDRESS);
  Wire.write(0x8F);  //APDS9960_CONTROL
  Wire.write(0x03);
  Wire.endTransmission();
  if (type == COLOR){  // Additional setup commands for the sensor
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xAB);  //APDS9960_GCONF4
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xE7);  //APDS9960_AICLEAR
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.write(0x03);
    Wire.endTransmission();
  }
  else if (type == GESTURE){   // Set up for gesture sensing
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xAB);  //APDS9960_GCONF4
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xE7);  //APDS9960_AICLEAR
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x92);  //APDS9960_ID
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    control = Wire.read();

    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x83);  //APDS9960_WTIME
    Wire.write(0xFF);  //APDS9960_GFIFO_R
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x8E);  //APDS9960_PPULSE
    Wire.write(0x89);  //APDS9960_PILT
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x90);  //APDS9960_CONFIG2
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    control = Wire.read();

    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x90);  //APDS9960_CONFIG2
    Wire.write(0x31);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xAB);  //APDS9960_GCONF4
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xAB);  //APDS9960_GCONF4
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xAB);  //APDS9960_GCONF4
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    control = Wire.read();

    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xAB);  //APDS9960_GCONF4
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    control = Wire.read();

    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    control = Wire.read();

    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.write(0x09);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    control = Wire.read();

    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.write(0x0D);
    Wire.endTransmission();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    control = Wire.read();

    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.write(0x4D);
    Wire.endTransmission();
  }
}

// The readColor function reads the color data from the APDS9960 sensor
// It reads the raw data for red, green, and blue channels and combines them
// into a single color value, returning the color detected.
int APDS9960::readColor(){
  Wire.beginTransmission(APDS_ADDRESS);
  Wire.write(0x93);  //APDS9960_STATUS
  Wire.endTransmission();
  Wire.requestFrom(APDS_ADDRESS, 1);
  int control = Wire.read();
  // If color data is available, read the red, green, and blue values
  if (control == 0x11){
    //Red channel
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x96);  //APDS9960_RDATAL
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    red_L = Wire.read();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x97);  //APDS9960_RDATAH
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    red_H = Wire.read();
    //Green channel
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x98);  //APDS9960_GDATAL
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    green_L = Wire.read();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x99);  //APDS9960_GDATAH
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    green_H = Wire.read();
    // Blue channel
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x9A);  //APDS9960_BDATAL
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    blue_L = Wire.read();
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x9B);  //APDS9960_BDATAH
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    blue_H = Wire.read();

    // Combine low and high bytes for each color channel
    red = red_L + red_H * 256;
    green = green_L + green_H * 256;
    blue = blue_L + blue_H * 256;

    // Return the detected color based on the highest value between red, green, and blue
    if ((red >= 500) && (green >= 500) && (blue >= 500))  // No significant color detected
      return APDS_NONE;
    if ((red > green) && (red > blue))
      return APDS_RED;
    if ((green > red) && (green > blue))
      return APDS_GREEN;
    if ((blue > green) && (blue > red))
      return APDS_BLUE;
  }
  else
    return APDS_NONE;  //if no valid color is detected
}

// The readGesture function reads the gesture data from the APDS9960 sensor.
// It checks the FIFO buffer for the gesture data and analyzes the movement direction.
// Returns UP, DOWN, LEFT, RIGHT, or NONE based on the detected gesture.
int APDS9960::readGesture(){
    // Check if valid gesture data is available
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xAF);  //APDS9960_GSTATUS
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    control = Wire.read();

    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0x80);  //APDS9960_ENABLE
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    control2 = Wire.read();

    if ((control == 0) && (control2 == 0))  // if no valid gesture data is detected
      return APDS_NONE;

    delay(3);
    // Check FIFO (First-In-First-Out) buffer status
    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xAF);  // Gesture FIFO status
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    gstatus = Wire.read();

    Wire.beginTransmission(APDS_ADDRESS);
    Wire.write(0xAE);  // FIFO level register
    Wire.endTransmission();
    Wire.requestFrom(APDS_ADDRESS, 1);
    fifo_level = Wire.read();

    if ((gstatus == 0) && (fifo_level == 0))
      return APDS_NONE;

    // Read data from the FIFO buffer
    for (int i=0; i<32; i++){
      // Read up gesture data
      Wire.beginTransmission(APDS_ADDRESS);
      Wire.write(0xFC);  //APDS9960_GFIFO_U
      Wire.endTransmission();
      Wire.requestFrom(APDS_ADDRESS, 1);
      data = Wire.read();
      u_data[i] = data;
      // Read down gesture data
      Wire.beginTransmission(APDS_ADDRESS);
      Wire.write(0xFD);  //APDS9960_GFIFO_D
      Wire.endTransmission();
      Wire.requestFrom(APDS_ADDRESS, 1);
      data = Wire.read();
      d_data[i] = data;
      // Read left gesture data
      Wire.beginTransmission(APDS_ADDRESS);
      Wire.write(0xFE);  //APDS9960_GFIFO_L
      Wire.endTransmission();
      Wire.requestFrom(APDS_ADDRESS, 1);
      data = Wire.read();
      l_data[i] = data;
      // Read right gesture data
      Wire.beginTransmission(APDS_ADDRESS);
      Wire.write(0xFF);  //APDS9960_GFIFO_R
      Wire.endTransmission();
      Wire.requestFrom(APDS_ADDRESS, 1);
      data = Wire.read();
      r_data[i] = data;

      delay(5);
    }

    // Initialize variables to store the first and last significant gesture data
    int u_first = 0, d_first = 0, l_first = 0, r_first = 0;
    int u_last = 0, d_last = 0, l_last = 0, r_last = 0;

    // Find the first significant gesture data
    for (int i=0; i<32; i++){
      if ((u_data[i] > APDS9960_THRESHOLD_OUT) && (d_data[i] > APDS9960_THRESHOLD_OUT) && (l_data[i] > APDS9960_THRESHOLD_OUT) && (r_data[i] > APDS9960_THRESHOLD_OUT)){
          u_first = u_data[i];
          d_first = d_data[i];
          l_first = l_data[i];
          r_first = r_data[i];
          break;
      }
    }

    if ((u_first == 0) ||  (d_first == 0) || (l_first == 0) || (r_first == 0))  // Exit if no valid starting point is found
      return  APDS_NONE;

    // Find the last significant gesture data
    for (int i = 31; i >= 0; i--){
      if ((u_data[i] > APDS9960_THRESHOLD_OUT) && (d_data[i] > APDS9960_THRESHOLD_OUT) && (l_data[i] > APDS9960_THRESHOLD_OUT) && (r_data[i] > APDS9960_THRESHOLD_OUT)){
        u_last = u_data[i];
        d_last = d_data[i];
        l_last = l_data[i];
        r_last = r_data[i];
        break;
      }
    }

    //Calculate ratios for up/down and left/right gestures
    float ud_ratio_first = ((u_first - d_first) * 100.0) / (u_first + d_first);
    float lr_ratio_first = ((l_first - r_first) * 100.0) / (l_first + r_first);
    float ud_ratio_last = ((u_last - d_last) * 100.0) / (u_last + d_last);
    float lr_ratio_last = ((l_last - r_last) * 100.0) / (l_last + r_last);

    float ud_delta = ud_ratio_last - ud_ratio_first;
    float lr_delta = lr_ratio_last - lr_ratio_first;

    int APDS9960_ud_count = 0;
    int APDS9960_lr_count = 0;

    if (ud_delta >= APDS9960_SENSITIVITY_1) {
      APDS9960_ud_count = 1;
    } else if (ud_delta <= -APDS9960_SENSITIVITY_1) {
      APDS9960_ud_count = -1;
    }

    if (lr_delta >= APDS9960_SENSITIVITY_1) {
      APDS9960_lr_count = 1;
    } else if (lr_delta <= -APDS9960_SENSITIVITY_1) {
      APDS9960_lr_count = -1;
    }

    // Return detected gesture based on direction
    if (APDS9960_ud_count == -1 && APDS9960_lr_count == 0) 
      return APDS_UP;
    if (APDS9960_ud_count == 1 && APDS9960_lr_count == 0) 
      return APDS_DOWN;
    if (APDS9960_ud_count == 0 && APDS9960_lr_count == 1) 
      return APDS_RIGHT;
    if (APDS9960_ud_count == 0 && APDS9960_lr_count == -1) 
      return APDS_LEFT;

    return APDS_NONE;
}

/*****************************
CY8CMBR3116 LIBRARY
******************************/
// Constructor for the CY8CMBR3116 class
CY8CMBR3116::CY8CMBR3116(){}

// The 'init' function sets up the CY8CMBR3116 touch sensor by initializing communication
// and sending the necessary configuration data to the sensor via I2C commands.
// This includes setting up the sensor registers for proximity detection and button states.
void CY8CMBR3116::init() {
    Wire.begin();

    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(0x00);
    Wire.write(0xFF);
    Wire.write(0x7F);
    Wire.write(0xFE);
    Wire.write(0x7F);
    for(int i=0; i<8 ; i++)
      Wire.write(0x00);
    Wire.write(0x0E);
    for(int i=0; i<15 ; i++)
      Wire.write(0x84);
    Wire.write(0x03);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(0x1F);
    for(int i=0; i<7 ; i++)
      Wire.write(0x00);
    Wire.write(0x01);
    Wire.write(0x81);
    Wire.write(0x06);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0xFF);
    Wire.write(0xF0);
    Wire.write(0x02);
    for(int i=0; i<16 ; i++)
      Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(0x3E);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    for(int i=0; i<8 ; i++)
      Wire.write(0xFF);
    for(int i=0; i<4 ; i++)
      Wire.write(0x00);
    Wire.write(0x03);
    Wire.write(0x01);
    Wire.write(0x58);
    Wire.write(0x00);
    Wire.write(0x37);
    Wire.write(0x06);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x0A);
    for(int i=0; i<7 ; i++)
      Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(0x5D);
    for(int i=0; i<31 ; i++)
      Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(0x7C);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x87);
    Wire.write(0x04);
    Wire.endTransmission();

    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(0x86);
    Wire.write(0x02);
    Wire.endTransmission();
    delay(500);
    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(0x86);
    Wire.write(0xFF);
    Wire.endTransmission();
    delay(500);
}

// This function reads the touch sensor data to detect button presses and proximity
// It sends and receives data over I2C to read button statuses and proximity sensor values
// The function updates the status and returns the corresponding button or proximity value.
int CY8CMBR3116::readTouch() {
    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(PROX_STAT); 
    Wire.endTransmission();
    Wire.requestFrom(CHIP_ADDRESS, 1);
    val[0] = Wire.read();

    Wire.beginTransmission(CHIP_ADDRESS);
    Wire.write(BUTTON_STATUS); 
    Wire.endTransmission();
    Wire.requestFrom(CHIP_ADDRESS, 2);
    buff[0] = Wire.read();
    buff[1] = Wire.read();

    if ((val[0] & 0x01) != 0) {
        if (++proximityCounter > 30) {
            proximityStatus = 1;
            proximityCounter = 0;
        }
    } else {
        proximityCounter = 0;
        proximityStatus = 0;
    }

    // Map button states to corresponding button values
    if (buff[0] == 2)  // X Button
      value = BUTTON_X;
    if (buff[0] == 4)  // Y Button
      value = BUTTON_Y;
    if (buff[0] == 16)  // Up Button
      value = UP_BUTTON;
    if (buff[0] == 32)  // Right Button
      value = RIGHT_BUTTON;
    if (buff[0] == 64)  // Down Button
      value = DOWN_BUTTON;
    if (buff[0] == 128)  // Left Button
      value = LEFT_BUTTON;
    
    if (buff[0] == 8)  // C1 Button
      value = BUTTON_C1;
    if (buff[1] == 64)  // D Button
      value = BUTTON_D;
    if (buff[1] == 32)  // E Button
      value = BUTTON_E;
    if (buff[1] == 16)  // F Button
      value = BUTTON_F;
    if (buff[1] == 8)  // G Button
      value = BUTTON_G;
    if (buff[1] == 4)  // A Button
      value = BUTTON_A;
    if (buff[1] == 2)  // B Button
      value = BUTTON_B;
    if (buff[1] == 1)  // C2 Button
      value = BUTTON_C2;
      
    // If no button is pressed, return value 0
    if ((buff[0] == 0) && (buff[1] == 0))
      value = 0;

    return value;
}
