// 868mhz / 915Mhz OLED LoRa sender

/**********************
 * Add warning logic for (INFO=display, WARN=quiet beep, ERR=loud beep)
 *   Configurable temp limits (ERR)
 *   Lost packets [rolling counter of missed counted packet] (INFO)
 *   Last packet timeout (WARN)
 *   Low RSSI (INFO)
 * Add beeper
 * Change temp/hum to floats
 * Add prg button logic to toggle display
 * Synchronized deep sleep? (receiver has wider window)
 * Enlarge fonts
 *********************/

#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <LoRa.h>
#include <SPI.h>
#include <Timer.h>

// LoRa radio pins and config
#define SCK 5           // GPIO5 - SX1278's SCK
#define MISO 19         // GPIO19 - SX1278's MISO
#define MOSI 27         // GPIO27 - SX1278's MOSI
#define SS 18           // GPIO18 - SX1278's CS
#define RST 14          // GPIO14 - SX1278's RESET
#define DI0 26          // GPIO26 - SX1278's IRQ (interrupt request)
#define LORA_BAND 915E6 // 868E6 for 868 Mhz or 915E6 for 915 Mhz

// OLED pins
#define OLED_I2C_ADDR 0x3C
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

// All other hardware pins
#define SPEAKER_PIN 36 // GPIO of the piezo speaker
#define ONBOARD_LED 2

// Constants
#define DEBUG false                 // DANGER causes wdt panics! // Increase verbosity on serial
#define DEBUG_DISPLAY true          // Detailed stats on OLED instead of just error
#define UPDATE_TIMER_INTERVAL 1000L // Process packets and update the display every x ms
#define SIGNAL_LOST_TIMEOUT 30      // Turn board LED on when last packet came in longer than x seconds ago
#define MAX_DELAY 5
#define MIN_RSSI -100
#define MAX_TEMP 75
#define MIN_TEMP 65
#define TEMP_OFFSET -8
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// State vars
bool show_display = true;
bool beep_on_packet = false;               // this is probably bad in a loop()
String last_packet_raw = "";               // The last packet we received // TODO should no longer be a global
int last_rssi = 0;                         // The RSSI level of the last packet
unsigned long last_packet_time = millis(); // The timestamp of the last packet
unsigned long last_delay = 0;              // How many seconds ago did we see the last packet
struct packet {
    bool parsed_ok = false;
    int rssi = 0;
    int temperature;
    int humidity;
    int counter = 0;
} last_packet;
String last_error;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
Timer update_timer; // For processing packets

void timestamped_log(String text, bool eol = true, bool timestamp = true) {
    if (DEBUG) {
        if (eol) {
            Serial.println(text);
        } else {
            Serial.print(text);
        }
    }
}

// Beep the buzzer on channel 0
void beep(int ms = 250, int tone = 3000) {
    return; // XXX restore this

    if (beep_on_packet) {
        ledcWriteTone(0, tone);
        delay(ms); // TODO make this async
        ledcWriteTone(0, 0);
    }
}

void long_beep() {
    beep(3000);
}

void short_beep() {
    beep(100);
}

// Extract realtime data from the packet payload
void parse_payload(packet &parsed_packet, String json) {
    StaticJsonBuffer<256> jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(json);

    timestamped_log("Attempting to parse '" + json + "'");

    if (root.success()) {
        parsed_packet.parsed_ok = true;
        parsed_packet.counter = root["c"];
        parsed_packet.temperature = int(root["t"]) + TEMP_OFFSET; // Calculate real temperature at receive-time
        parsed_packet.humidity = root["h"];
        parsed_packet.rssi = last_rssi;

        if (root["loc"].success()) {
        }

        timestamped_log("Parsing succeeded of " + json + ".");
    } else {
        timestamped_log("Parsing failed of " + json + ".");

        parsed_packet.parsed_ok = false;
    }
}

// Determine whether a packet meets our basic validity criteria
bool is_valid_packet(String packet) {
    return (strstr(packet.c_str(), "\"c\":") != NULL);
}

// Read LoRa packet into a buffer when we receive one
void process_lora_packet(int packetSize) {
    int max_packet_size = 256;
    String packet = "";
    int rssi = LoRa.packetRssi();

    // Read packet
    for (int i = 0; (i < packetSize) && (i < max_packet_size); i++) {
        packet += (char)LoRa.read();
    }

    // Filter out garbage on the open channel
    if (is_valid_packet(packet)) {
        last_packet_raw = packet;
        last_rssi = rssi;
        last_packet_time = millis();

        // Parse and store the last packet in a global struct for use everywhere
        parse_payload(last_packet, last_packet_raw);

        timestamped_log("Valid packet: " + String(packet));
    } else {
        // FIXME we can still see corruption in parts of the packet beyond the mandatory c parameter
        timestamped_log("Ignoring corrupt packet.");
    }
}

// Refresh the OLED
void update_display() {
    int y;

    display.clearDisplay();
    y = 0;

    // if (last_error = "") {
    display.setTextSize(2);
    display.setCursor(0, y);

    display.print("Temp:");
    display.setCursor(SCREEN_WIDTH / 2, y);
    display.print(last_packet.temperature, 1);
    display.print("F");

    y += 16;
    display.setCursor(0, y);
    display.print("Hum:");
    display.setCursor(SCREEN_WIDTH / 2, y);
    display.print(last_packet.humidity, 1);
    display.print("%");

    display.setTextSize(1);

    if (DEBUG_DISPLAY) {
        y += 18;
        display.setCursor(0, y);
        display.print("Uptime: ");
        display.setCursor(SCREEN_WIDTH / 2, y);
        display.print(last_packet.counter);
        display.println(" sec");

        y += 8;
        display.setCursor(0, y);
        display.print("Last pkt:");
        display.setCursor(SCREEN_WIDTH / 2, y);
        display.print(last_delay);
        display.println(" s ago");

        y += 8;
        display.setCursor(0, y);
        display.print("RSSI:");
        display.setCursor(SCREEN_WIDTH / 2, y);
        display.print(last_rssi);
        display.println(" dB");
    } else {
        y += 20;
        display.setCursor(0, y);
        display.setTextSize(1);

        display.print(last_error);
    }

    display.display();
}

// MUST BE LOW-LATENCY! No serial logging!
// Display the contents of the last packet on the display
void process_data() {
    // Compute when we last saw a packet
    last_delay = (millis() - last_packet_time) / 1000;
    last_error = "";

    String log_text = "Last packet: " + String(last_delay) + "s ago";
    //  + " / Heap: " + String(esp_get_free_heap_size());

    // TODO check for new location only
    // If we have a different location and it's a valid one, let's record it
    if (last_packet.parsed_ok) {
        digitalWrite(ONBOARD_LED, HIGH);

        timestamped_log(log_text);

        if (last_packet.temperature > MAX_TEMP || last_packet.temperature < MIN_TEMP) {
            last_error = "Temperature is outside of range";

            long_beep();
        }

    } else {
        last_error = "Couldn't parse packet";
    }

    if (last_rssi < MIN_RSSI) {
        last_error = "WARN: RSSI is low";

        short_beep();
    }

    if (last_delay > MAX_DELAY) {
        last_error = "WARN: last packet was long ago";

        short_beep();
    }

    if (last_error != "") {
        timestamped_log(last_error);
    }

    // Update the on-board display with new info
    if (show_display) {
        update_display();
        digitalWrite(ONBOARD_LED, LOW);
    }
}

// Initialize the LoRa radio
void init_lora() {
    timestamped_log("Initializing LoRa Radio...");

    // Init the radio
    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DI0);
    if (!LoRa.begin(LORA_BAND)) {
        timestamped_log("LoRa init failed!");

        ESP.restart();
    }

    timestamped_log("LoRa init ok.");
}

// Initialize the OLED
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
    display.setRotation(2);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Starting up...");
    display.display();

    // display.flipScreenVertically();
    // display.setTextAlignment(TEXT_ALIGN_LEFT);
    // display.setFont(ArialMT_Plain_10);
}

// Toggle display on every push of the PRG button
void handle_prg_button_push() {
    show_display != show_display;
}

// Initialize system
void setup() {
    Serial.begin(115200); // Note the high baud rate when connecting your serial monitor
    while (!Serial)
        ;

    pinMode(ONBOARD_LED, OUTPUT);

    pinMode(0, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(0), handle_prg_button_push, FALLING);

    // For the PWM-modulated piezo speaker
    // ledcSetup(0, 1000, 8);
    // ledcAttachPin(SPEAKER_PIN, 0);

    beep(100, 500); // Quick beep on power-on

    init_display();

    init_lora();

    // Register the receiver callback
    LoRa.onReceive(process_lora_packet);
    LoRa.receive();

    // Process new data and update the display regularly
    update_timer.every(UPDATE_TIMER_INTERVAL, process_data);

    // ntp_client.update();
    timestamped_log("System started.");

    beep(250, 2000); // Quick beep on full init
}

// Main loop
void loop() {
    update_timer.update();
}
