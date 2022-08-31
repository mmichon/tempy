// 868mhz / 915Mhz OLED LoRa receiver

/************

TODO
x Read temp sensor
* Print case with hole, calibrate sensor
* Hass wifi client
* OTA updates
x Edit display text
x Encode/decode sensor values

************/
#include "SHTSensor.h"

#include "SHTSensor.h"
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <LoRa.h>
#include <SPI.h>
// #include <SSD1306.h>
#include <Timer.h>
#include <Wire.h>

// LoRa pins
#define SCK 5   // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18   // GPIO18 - SX1278's CS
#define RST 14  // GPIO14 - SX1278's RESET
#define DI0 26  // GPIO26 - SX1278's IRQ (interrupt request)

// OLED pins
#define OLED_I2C_ADDR 0x3C
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// SHT pins
#define I2C_SDA 21
#define I2C_SCL 22

#define ONBOARD_LED 2         // Onboard LED
#define LORA_BAND 915E6       // or 915E6
#define PIN_WAKEUP GPIO_NUM_0 // Tilt switch
#define USE_DISPLAY true      // Whether to sleep the OLED or not
#define DEBUG true            // Extra verbosity
#define SEND_INTERVAL 5000L   // Send packets every 5 seconds

// SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL); // 128x64 OLED Display on TTGO board
// SHTSensor sht(SHTSensor::SHT3X);
TwoWire Wire2 = TwoWire(1);
SHTSensor sht;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// State vars
bool show_display = true;
unsigned long sensor_last_read = 0; // When the GPS was last fixed
unsigned int counter = 0;           // Monotonically increasing sent packet counter
Timer get_sensor_values_and_send_timer;
int get_sensor_values_and_send_timer_index;

// Updates the OLED with new info
void update_display(float t, float h) {
    // init_display();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);

    if (!isnan(t)) { // check if 'is not a number'
        display.setCursor(0, 20);
        display.print("Temp:");
        display.setCursor(50, 20);
        display.print(t, 1);
        display.print("F");
    } else {
        Serial.println("Failed to read temperature");
        display.setCursor(0, 20);
        display.print("Temperature Error");
    }

    if (!isnan(h)) { // check if 'is not a number'
        display.setCursor(0, 30);
        display.print("Hum:");
        display.setCursor(50, 30);
        display.print(h, 1);
        display.print("%");
    } else {
        Serial.println("Failed to read humidity");
        display.setCursor(0, 30);
        display.print("Humidity Error");
    }

    display.setCursor(0, 50);
    display.print("Uptime:");
    display.setCursor(50, 50);
    display.print(counter);
    display.print(" sec");

    display.display();
}

// Gets latest GPS location and sends it as a LoRa packet
void get_sensor_values_and_send() {
    char pkt_buffer[256];
    float temperature;
    float humidity;

    if (sht.readSample()) {
        temperature = sht.getTemperature() * 1.8 + 32; // convert from C->F
        humidity = sht.getHumidity();

        // if (DEBUG) {
        //     Serial.print("\t");
        //     Serial.print(humidity, 1);
        //     Serial.print("\t\t");
        //     Serial.print(temperature, 1);
        // }
    } else {
        if (DEBUG) {
            Serial.print("ERROR reading SHT3X-D\n");
        }
    }

    if (USE_DISPLAY && show_display) {
        update_display(temperature, humidity);
        /* display.clear();
        //display.setTextAlignment(TEXT_ALIGN_CENTER);
        //display.setFont(ArialMT_Plain_24);
        //display.drawStringMaxWidth(64, 15, 128, String(counter));
        display.drawString(64, 15, String(counter));
        //display.print(String(counter));
        display.display();
         */
    }

    // Prep the packet
    unsigned long sensor_last_read_us = (micros() - sensor_last_read) / 1000000L;

    // TODO add GPS timestamp, batt voltage, sat age to the packet
    sprintf(pkt_buffer, "{\"c\":\"%d\",\"t\":\"%f\",\"h\":\"%f\",\"lr\":\"%lu\"}",
            counter,
            temperature,
            humidity,
            sensor_last_read_us);

    if (DEBUG)
        Serial.println("Sending packet: " + String(pkt_buffer));

    // Send packet over LoRa
    LoRa.beginPacket();
    LoRa.print(pkt_buffer);
    LoRa.endPacket();

    // Increment packet counter
    counter++;
}

void init_display() {
    // reset OLED display via software
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);

    // initialize OLED
    Wire.begin(OLED_SDA, OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
        while (1) {
        }; // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Starting up...");
    display.display();

    // display.flipScreenVertically();
    // display.setTextAlignment(TEXT_ALIGN_LEFT);
    // display.setFont(ArialMT_Plain_10);
}

// Initialize the LoRa radio
void init_lora() {
    Serial.println();
    Serial.println("Initializing LoRa radio...");

    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DI0);
    if (!LoRa.begin(LORA_BAND)) {
        Serial.println("LoRa init failed!");

        // Die if we can't init
        while (true)
            ;
    }

    Serial.println("LoRa init ok.");
}

void init_sht3xd() {
    Wire2.begin(I2C_SDA, I2C_SCL);

    if (sht.init(Wire2)) {
        sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
        Serial.println("SHT3X-D init succeeded");
    } else {
        Serial.println("SHT3X-D init failed");
    }
}

// Toggle display on every push of the PRG button
void handle_prg_button_push() {
    show_display != show_display;
}

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    delay(1000);

    pinMode(ONBOARD_LED, OUTPUT);

    pinMode(0, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(0), handle_prg_button_push, FALLING);

    if (USE_DISPLAY) {
        init_display();
    } else {
        // TODO: turn off display charge pump
        // display.sendCommand(0x8D); // into charger pump set mode
        // display.sendCommand(0x10); // turn off charger pump
        // display.sendCommand(0xAE); // set OLED sleep
    }

    init_sht3xd();

    init_lora();

    get_sensor_values_and_send_timer_index = get_sensor_values_and_send_timer.every(SEND_INTERVAL, get_sensor_values_and_send);
}

void loop() {
    get_sensor_values_and_send_timer.update();

    // TODO Fix wakeup pin logic
    /*
    //  Night night
    if (DEBUG) Serial.println("Going to sleep...");
    if (esp_sleep_enable_ext0_wakeup(PIN_WAKEUP, 0) == ESP_OK) {
        esp_deep_sleep_start();
        if (DEBUG) Serial.println("I went to sleep."); // should never see this
    } else {
        Serial.println("Couldn't go to sleep.");
    }
*/
}
