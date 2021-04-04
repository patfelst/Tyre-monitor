#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <FastLED.h>
#include <OneButton.h>
#include <Wire.h>

#include "Adafruit_MCP9600.h"
#include "hsc_ssc_i2c.h"

#define firmware_version      "v0.1"
#define firmware_date         __DATE__
#define firmware_ver_and_date firmware_version ", " firmware_date

#define NUM_TYRE_SAMPLES        4    // The number of samples to take upon button click
#define TYRE_SAMPLE_PERIOD_MS   250  // Period between each pressure/temperature sample
#define PRESSURE_THRESHOLD_PSIG 0.2  // Threshold above which pressure is reported over BLE instead of temperature
uint32_t last_sample_time = 0;
uint32_t message_update_time = 0;
bool never_connected = true;
void tyre_deep_sleep();
void send_ble_data();
void send_ble_data_2();

// User push button
#define BUTTON_GPIO_PIN 33
OneButton button = OneButton(
    BUTTON_GPIO_PIN,  // Input pin for the button
    true,             // Button is active LOW
    true              // Enable internal pull-up resistor
);
void button_shortpress();
void button_longpress_started();
void button_longpress_stopped();
void button_during_long_press();

// Bluetooth Low Energy
#define BLE_CONNECTION_TIMEOUT_SEC 20                                      // 10 seconds
#define SERVICE_UUID               "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX     "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // UART Receive UUID
#define CHARACTERISTIC_UUID_TX     "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // UART Transmit UUID
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool BLE_Connected = false;
bool BLE_Advertising = false;
uint8_t ble_tx_type = 1;  // What format to output the data over BLE: 0 = String / 1 = CSV / 2 = Integer array

// MCP9600 Thermocouple
#define MCP9600_I2C_ADDR (0x67)
Adafruit_MCP9600 mcp;
void read_thermocouple(float *thermocouple, float *ambient);

// TinyPico APA102 RGB LED
#define NUM_LEDS                   1
#define RGBLED_PWR                 13
#define RGBLED_DATA_PIN            2
#define RGBLED_CLK_PIN             12
#define LED_Black                  0x000000
#define LED_Green                  0x008000
#define LED_Red                    0xFF0000
#define LED_Blue                   0x0000FF
#define LED_Yellow                 0xFFFF00
#define LED_DarkOrange             0xFF8C00
#define LED_Purple                 0x800080
#define LED_Fuschia                0xFF00FF
#define LED_Cyan                   0x00FFFF
#define RGB_LED_BRIGHTNESS_PERCENT 50
CRGB leds[NUM_LEDS];
void set_LED_colour(uint32_t color, uint8_t brightness_percent);
void flash_LED(uint32_t color, uint8_t brightness_percent, uint8_t freq, int16_t flash_number);

// Setup Honywell HSCMAND060PA3A3 60psi absolute pressure sensor
// see hsc_ssc_i2c.h for a description of these values
#define SLAVE_ADDR 0x38
uint8_t hsc_status;
float atmos_pressure;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    never_connected = false;
    BLE_Connected = true;
    set_LED_colour(LED_Blue, RGB_LED_BRIGHTNESS_PERCENT);
  };

  void onDisconnect(BLEServer *pServer) {
    BLE_Connected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received UART value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);
      Serial.println();
      Serial.println("*********");

      // Look for command to change BLE output format
      if (rxValue.length() == 3 && (rxValue[0] == '@')) {
        ble_tx_type = rxValue[1] - '0';
        switch (ble_tx_type) {
          case 0:
            Serial.println("Switching to READABLE STRING\n");
            break;

          case 1:
            Serial.println("Switching to CSV FORMAT\n");
            break;

          case 2:
            Serial.println("Switching to INTEGER ARRAY\n");
            break;

          default:
            ble_tx_type = 0;
            break;
        }
      }
    }
  }
};

/*
  -----------------
  Arduino Setup
  -----------------
*/
void setup() {
  /********************************************************************
                  INITIALISE Serial Port For Debug Text
  ********************************************************************/
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(firmware_ver_and_date);
  Serial.println();

  /********************************************************************
              INITIALISE Honeywell HSC Pressure Sensor
  ********************************************************************/
  // I2C peripheral on GPIO pins SDA = 21, SCL = 22
  Wire.begin(21, 22);
  // Read Atmospheric pressure to calculate gague pressure later on
  hsc_status = read_hsc_absolute(SLAVE_ADDR, &atmos_pressure, nullptr);
  if (!hsc_status) Serial.printf("Atmospheric pressure read OK = %.2f psi\n", atmos_pressure);

  /********************************************************************
              INITIALISE APA102 RGB LED on TinyPico Board
  ********************************************************************/
  // Apply power to APA102 - active LOW
  pinMode(RGBLED_PWR, OUTPUT);
  digitalWrite(RGBLED_PWR, LOW);
  FastLED.addLeds<APA102, RGBLED_DATA_PIN, RGBLED_CLK_PIN, BGR>(leds, NUM_LEDS);
  flash_LED(LED_Green, RGB_LED_BRIGHTNESS_PERCENT, 6, 3);
  delay(400);
  set_LED_colour(LED_Red, RGB_LED_BRIGHTNESS_PERCENT);

  /********************************************************************
                INITIALISE MCP9600 Thermocouple Interface
  ********************************************************************/
  if (!mcp.begin(MCP9600_I2C_ADDR)) {
    Serial.println("Sensor not found. Check wiring!");
    while (1) continue;
  }

  // Set Thermocouple ADC resolution set to 18 bits
  mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
  log_d("ADC thermocouple resolution set to 18 bits");

  // Set Thermocouple to K type
  mcp.setThermocoupleType(MCP9600_TYPE_K);
  log_d("Thermocouple type set to K type");

  // Set Thermocouple Filter coefficient
  mcp.setFilterCoefficient(3);
  log_d("Filter coefficient value set to: %d", mcp.getFilterCoefficient());

  // Enable the Thermocouple
  mcp.enable(true);

  /*
  ********************************************************************
  *************** INITIALISE BLUETOOTH LOW ENERGY ********************
  ********************************************************************
  */

  // Create the BLE Device
  BLEDevice::init("Tyre Monitor");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.printf("BLE Advertising - Waiting %d seconds for client connection...\n", BLE_CONNECTION_TIMEOUT_SEC);

  /*
  ********************************************************************
  ********************* INITIALISE PUSH BUTTON  **********************
  ********************************************************************
  */
  //pinMode(BUTTON_GPIO_PIN, INPUT_PULLUP);
  button.attachClick(button_shortpress);
  button.attachDuringLongPress(button_during_long_press);
  button.attachLongPressStart(button_longpress_started);
  button.attachLongPressStop(button_longpress_stopped);
  button.setClickTicks(50);     // short click after this period
  button.setDebounceTicks(20);  // debounce period
  button.setPressTicks(500);    // long press after this period
}

/*
  -----------------
  Send Tyre monitor data via BLE UART service
  Send pressure if present, else send temperature
  -----------------
*/
void send_ble_data_2() {
  char txt[40];
  float thermocouple_temp;
  float gauge_pressure;
  uint16_t send_value = 0;

  if (BLE_Connected) {
    // LED is green while measuring
    set_LED_colour(LED_Green, RGB_LED_BRIGHTNESS_PERCENT);

    // Initial measurement to see if pressure sensor is attached to tyre
    gauge_pressure = read_hsc_gauge(SLAVE_ADDR, atmos_pressure);

    if (gauge_pressure > PRESSURE_THRESHOLD_PSIG) {
      for (uint8_t n = 0; n < NUM_TYRE_SAMPLES; n++) {
        gauge_pressure = read_hsc_gauge(SLAVE_ADDR, atmos_pressure);
        // SEND CSV FORMATTED ASCII FOR ADAFRUIT BLUEFRUIT CONNECT APP
        send_value = (uint16_t)(gauge_pressure * 100.0);
        sprintf(txt, "%d,\n", send_value);
        Serial.printf("CSV format: Gauge Pressure = %d [dec] = %.2f psi\n", send_value, gauge_pressure);
        pTxCharacteristic->setValue(txt);
        pTxCharacteristic->notify();
        delay(TYRE_SAMPLE_PERIOD_MS);
      }
    } else {
      for (uint8_t n = 0; n < NUM_TYRE_SAMPLES; n++) {
        read_thermocouple(&thermocouple_temp, nullptr);
        // SEND CSV FORMATTED ASCII FOR ADAFRUIT BLUEFRUIT CONNECT APP
        send_value = (uint16_t)(thermocouple_temp * 100.0);
        sprintf(txt, "%d,\n", send_value);
        Serial.printf("CSV format: Temperature = %d [dec] = %.2f °C\n", send_value, thermocouple_temp);
        pTxCharacteristic->setValue(txt);
        pTxCharacteristic->notify();
        delay(TYRE_SAMPLE_PERIOD_MS);
      }
    }

    // Set LED colour back to Blue
    if (BLE_Connected)
      set_LED_colour(LED_Blue, RGB_LED_BRIGHTNESS_PERCENT);
  }
}

/*
  -----------------
  Callback function for Button short press
  -----------------
*/
void button_shortpress() {
  send_ble_data_2();
}

/*
  -----------------
  Callback function for Button long press started
  -----------------
*/
void button_longpress_started() {
}

/*
  -----------------
  Callback function for Button during a long press
  -----------------
*/
void button_during_long_press() {
  flash_LED(LED_Red, RGB_LED_BRIGHTNESS_PERCENT, 6, 5);
}

/*
  -----------------
  Callback function for Button long press stopped
  -----------------
*/
void button_longpress_stopped() {
  delay(1500);
  tyre_deep_sleep();
}

/*
  -----------------
  Arduino Main loop
  -----------------
*/
void loop() {
  delay(2);  // Give RTOS time to do stuff
  button.tick();

  // At boot up, stay awake for a while to allow phone to connect
  if (never_connected) {
    if (millis() - message_update_time > 500) {
      message_update_time = millis();
      log_i("No BLE connection for %.1f sec\n", (float)(millis()) / 1000.0);
    }
    if (millis() > (BLE_CONNECTION_TIMEOUT_SEC * 1000))
      tyre_deep_sleep();
  }

  // If just disconnected from phone, go straight to sleep
  if (!BLE_Connected && !never_connected)
    tyre_deep_sleep();
}

/*
  -----------------
  Put Tyre monitor CPU into deep sleep
  -----------------
*/
void tyre_deep_sleep() {
  // Setup wake from sleep by button on GPIO-33
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);
  Serial.println("Going to sleep. Press button to wake. Zzzz...");
  esp_deep_sleep_start();
}

/*
  -----------------
  Send Tyre monitor data via BLE UART service
  -----------------
*/
void send_ble_data() {
  float gauge_pressure = 0.0;
  float thermocouple_temp = 0.0;
  char press_txt[30];
  if (ble_tx_type > 0)
    Serial.printf("Pressure=%.2f psi, Temperature %.2f°C\n", gauge_pressure, thermocouple_temp);

  switch (ble_tx_type) {
    case 0:
      // SEND AS READABLE STRING
      sprintf(press_txt, "P=%.2fpsi, T=%.2f°C\n", gauge_pressure, thermocouple_temp);
      Serial.printf("Readable string: %s", press_txt);
      pTxCharacteristic->setValue(press_txt);
      pTxCharacteristic->notify();
      break;

    case 1:
      // SEND CSV FORMATTED ASCII FOR ADAFRUIT BLUEFRUIT CONNECT APP
      sprintf(press_txt, "%d,%d\n", (uint16_t)(gauge_pressure * 100.0), (uint16_t)(thermocouple_temp * 100.0));
      Serial.printf("CSV format: %s", press_txt);
      pTxCharacteristic->setValue(press_txt);
      pTxCharacteristic->notify();
      break;

    case 2:
      // SEND AS INTEGER ARRAY
      uint8_t ble_send_array[4];
      uint16_t ble_val1_int16;
      uint16_t ble_val2_int16;

      ble_val1_int16 = (uint16_t)(gauge_pressure * 100.0);     // Pressure should always be positive 0.0 - 60.0 psi
      ble_val2_int16 = (uint16_t)(thermocouple_temp * 100.0);  // Temp should be 0 - 70°C

      ble_send_array[0] = ble_val1_int16;
      ble_send_array[1] = ble_val1_int16 >> 8;
      ble_send_array[2] = ble_val2_int16;
      ble_send_array[3] = ble_val2_int16 >> 8;

      Serial.printf("Integer Array: Pressure=%d dec=[0x%02X] [0x%02X]\n", ble_val1_int16, ble_send_array[0], ble_send_array[1]);
      Serial.printf("Integer Array: Temperat=%d dec=[0x%02X] [0x%02X]\n", ble_val2_int16, ble_send_array[2], ble_send_array[3]);

      pTxCharacteristic->setValue(ble_send_array, 4);
      pTxCharacteristic->notify();
      break;

    default:
      break;
  }
  Serial.println("");
}

/*
  -----------------
  Read MCP9600 Thermocouple
  -----------------
*/
void read_thermocouple(float *thermocouple, float *ambient) {
  if (thermocouple) {
    *thermocouple = mcp.readThermocouple();
    log_d("Thermocouple = %.2f °C", *thermocouple);
  }

  if (ambient) {
    *ambient = mcp.readAmbient();
    log_d("MCP9600 Ambient = %.2f °C", ambient);
  }
}

/*
  -----------------
  Set and display TinyPico RGB LED colour
  -----------------
*/
void set_LED_colour(uint32_t color, uint8_t brightness_percent) {
  leds[0] = color;
  uint8_t led_bright_pc = ((brightness_percent * 0xFF) / 100);
  leds[0] %= led_bright_pc;
  FastLED.show();
}

/*
  -----------------
  Flash the RGB LED on the PCB
  color - colour to flash
  brightness_percent - percentage brightness
  freq - freq in Hz to flash
  flash_number - number of flashes to do. -1 = forever
  -----------------
*/
void flash_LED(uint32_t color, uint8_t brightness_percent, uint8_t freq, int16_t flash_number) {
  uint32_t period = 500 / freq;  // Double the frequency of 1000ms period
  uint32_t count = flash_number;

  while (count > 0) {
    leds[0] = color;
    uint8_t led_bright_pc = ((brightness_percent * 0xFF) / 100);
    leds[0] %= led_bright_pc;
    FastLED.show();
    delay(period);
    leds[0] = LED_Black;
    FastLED.show();
    delay(period);
    count--;
  };
}