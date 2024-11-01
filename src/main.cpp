#include <Arduino.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <WiFi.h>
#include <time.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ezButton.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "DFRobot_RGBLCD1602.h"
#include "sensor.h"

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// The pin the button is wired to
#define BUTTON_PIN 23

// How many ms equates to a long button press
#define LONG_PRESS_MS 5000

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

// NTP server details
const char *ntpServer = "pool.ntp.org";
long gmtOffset_sec = 9 * 3600 + 30 * 60; // Default Adelaide timezone (UTC+9:30)
const int daylightOffset_sec = 3600;     // Daylight saving time offset (1 hour)

// todo: this might not be needed if getLocalTime() is used. Check later.
bool timeIsSynced = false;

// Custom parameter for time offset
char timeOffset[6] = "9.5"; // Default value

// LCD display is 16 characters and 2 lines. RGB address 0x6B is for LCD1602 v1.1
DFRobot_RGBLCD1602 lcd(0x6B, 16, 2);

// LCD display is 16 characters and 2 lines. RGB address 0x60 is for LCD1602 v1.0
// DFRobot_RGBLCD1602 lcd(0x60,16,2);

ezButton button(BUTTON_PIN); // create ezButton object that attaches to pin SWITCH_PIN;

// which sensor board is being used
int currentBoard = 0;

// Structure used to receive sensor data and to pass the data to the loop task via a RTOS queue
typedef struct message_t
{
    bool tIsValid;
    float temperature;

    bool pIsValid;
    unsigned pressure;

    bool hIsValid;
    unsigned humidity;

    unsigned readingId;
} message_t;

// RTOS queue settings
const int msg_queue_len = 1; // Size of msg_queue. Only one message is needed
QueueHandle_t msg_queue;

// what to display on the LCD. Default to Temperature
enum displayMode : char
{
    TemperatureMode = 0,
    PressureMode,
    HumidityMode
};
displayMode currentDisplayMode = TemperatureMode;

Sensor<float> temperatureSensor;
Sensor<unsigned> pressureSensor;
Sensor<unsigned> humiditySensor;

enum status : char
{
    ok = 0,
    failed
};

status getWiFiStatus()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return ok;
    }
    else
    {
        return failed;
    }
}

void loadTimeOffset()
{
    if (!SPIFFS.exists("/timeOffset.txt"))
    {
        Serial.println("Time offset file does not exist");
        return;
    }
    File file = SPIFFS.open("/timeOffset.txt", FILE_READ);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }
    String offset = file.readString();
    offset.trim();
    offset.toCharArray(timeOffset, sizeof(timeOffset));
    gmtOffset_sec = offset.toFloat() * 3600; // Convert hours to seconds
    file.close();
}

void saveTimeOffset(const char *offset)
{
    File file = SPIFFS.open("/timeOffset.txt", FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    file.print(offset);
    file.close();
}

void setupWiFi()
{
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

    // WiFiManager, Local initialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;

    // Custom parameter for time offset
    WiFiManagerParameter custom_time_offset("time_offset", "Time Offset (hours)", timeOffset, 6);
    wm.addParameter(&custom_time_offset);

    // Uncomment to reset settings - wipe stored credentials for testing
    wm.resetSettings();

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ("AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid

    wm.setTimeout(30);                    // 3 minutes
    res = wm.autoConnect("AutoConnectAP"); // anonymous ap

    if (!res)
    {
        Serial.println("Failed to connect");
        // Uncomment to restart the ESP device if connection fails
        // ESP.restart();
    }
    else
    {
        Serial.println("Connected to WiFi");
    }

    // Retrieve the time offset from the custom parameter
    String timeOffsetStr = custom_time_offset.getValue();
    gmtOffset_sec = timeOffsetStr.toFloat() * 3600; // Convert hours to seconds

    // Save the time offset to the file
    saveTimeOffset(timeOffsetStr.c_str());
}

void setupNTP()
{
    // Initialize NTP
    Serial.println("Waiting for NTP time sync...");

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    // Wait until time is set
    while (time(nullptr) < 100000)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    timeIsSynced = true;

    Serial.println("Time synchronized successfully");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

    // Determine if daylight saving time is in effect
    if (timeinfo.tm_isdst > 0)
    {
        Serial.println("Daylight Saving Time is in effect");
    }
    else
    {
        Serial.println("Standard Time is in effect");
    }
}

void readSensors(void *parameter)
{
    message_t msg;
    unsigned readingId = 0;

    // TEMP: Pause the task for 5 seconds so I can see the LCD update
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    while (1)
    {
        // todo: read the sensors
        //        msg.temperature = bme.readTemperature();
        //        msg.tIsValid = true;

        //        msg.pressure = bme.readPressure() / 100.0F;
        //        msg.pIsValid = true;

        //        msg.humidity = bme.readHumidity();
        //        msg.hIsValid = true;

        // todo: store the sensor data in a queue to be processed by the main loop
        msg.tIsValid = true;
        msg.temperature = random(-10, 31);

        msg.pIsValid = true;
        msg.pressure = random(1000, 1030);

        msg.hIsValid = true;
        msg.humidity = random(30, 70);

        msg.readingId = readingId++;

        xQueueSend(msg_queue, (void *)&msg, 10 / portTICK_PERIOD_MS);

        // Pause the task for 60 seconds. 10 s for testing only
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void DisplayStatus()
{
    if (getWiFiStatus() == ok)
    {
        lcd.setCursor(14, 0); // col,row
        lcd.print("OK");
    }
    else if (getWiFiStatus() == failed)
    {
        lcd.setCursor(14, 0);
        lcd.print("XX");
    }
}

void DisplayTemperature(Sensor<float> &ts)
{
    lcd.clear();
    lcd.printf("Temp:%3.1f %s", ts.GetValue(), ts.GetTime().c_str());
    lcd.setCursor(0, 1);
    lcd.printf("L:%3.1f H:%3.1f",
               ts.GetMin(),
               ts.GetMax());
    //   lcd.setCursor(15, 1);
    //   lcd.print(ts.GetName()[0]);
}

void DisplayPressure(Sensor<unsigned> &ps)
{
    lcd.clear();
    lcd.printf("Press:%4d %s", ps.GetValue(), ps.GetTime().c_str());
    lcd.setCursor(0, 1);
    lcd.printf("L:%4d H:%4d",
               ps.GetMin(),
               ps.GetMax());
    // lcd.setCursor(15, 1);
    // lcd.print(ps.GetName()[0]);
}

void DisplayHumidity(Sensor<unsigned> &hs)
{
    lcd.clear();
    lcd.printf("Hum:%2d %s", hs.GetValue(), hs.GetTime().c_str());
    lcd.setCursor(0, 1);
    lcd.printf("L:%2d H:%2d",
               hs.GetMin(),
               hs.GetMax());
    // lcd.setCursor(15, 1);
    // lcd.print(hs.GetName()[0]);
}

void setup()
{
    // Initialize Serial Monitor
    Serial.begin(115200);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    button.setDebounceTime(50); // set debounce time to 50 milliseconds

    /*
        bool status = bme.begin();
        // You can also pass in a Wire library object like &Wire2
        // status = bme.begin(0x76, &Wire2)
        if (!status) {
            Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
            Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
            Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
            Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
            Serial.print("        ID of 0x60 represents a BME 280.\n");
            Serial.print("        ID of 0x61 represents a BME 680.\n");
            while (1) delay(10);
        }
    */

    // Initialize SPIFFS
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    // Load the UTC time offset
    loadTimeOffset();

    // initialize the LCD
    lcd.init();
    lcd.print("WiFi...");

    setupWiFi();
    // todo: need to handle the failure to connect case

    if (WiFi.status() == WL_CONNECTED)
    {
        lcd.clear();
        lcd.print("TimeSync...");

        setupNTP();
        // todo: need to handle the failure to connect case

        // the time has been synced so we don't need the wifi anymore
        WiFi.disconnect(true, false);
    }

    lcd.clear();
    lcd.print("Data...");

    // Create the message queue
    msg_queue = xQueueCreate(msg_queue_len, sizeof(message_t));

    // this function is specific to ESP32 RTOS with core parameter added
    xTaskCreatePinnedToCore(
        readSensors,    // Function that should be called
        "Read Sensors", // Name of the task (for debugging)
        1024,           // Stack size (bytes)
        NULL,           // Parameter to pass
        1,              // Task priority
        NULL,           // Task handle
        app_cpu);       // which core to use
}

void loop()
{
    static unsigned long buttonPressedTime;

    // Is there any sensor data on the queue?
    message_t incomingReadings;
    if (xQueueReceive(msg_queue, (void *)&incomingReadings, 0) == pdTRUE)
    {
        if (incomingReadings.tIsValid)
        {
            Serial.printf("Temperature: %3.1f\n", incomingReadings.temperature);
            temperatureSensor.SetValue(incomingReadings.temperature, "Builtin");
        }

        if (incomingReadings.pIsValid)
        {
            Serial.printf("Pressure: %d\n", incomingReadings.pressure);
            pressureSensor.SetValue(incomingReadings.pressure, "Builtin");
        }

        if (incomingReadings.hIsValid)
        {
            Serial.printf("Humidity: %d\n", incomingReadings.humidity);
            humiditySensor.SetValue(incomingReadings.humidity, "Builtin");
        }

        Serial.printf("Reading ID: %d\n", incomingReadings.readingId);
        Serial.println();

        // todo: hack - display something on the LCD
        DisplayTemperature(temperatureSensor);
    }

    // check the pushbutton
    // MUST call the loop() function first, the library requires it.
    button.loop();

    if (button.isPressed())
    {
        Serial.println("The button is pressed");
        buttonPressedTime = millis();
    }

    if (button.isReleased())
    {
        Serial.println("The button is released");
        if (millis() - buttonPressedTime > LONG_PRESS_MS)
        {
            // long press signifies the user wants to enter wifi credentials
            Serial.println("Long press");
        }
        else
        {
            // short press signifies the user wants to change mode, ie, from Temperature to Pressure
            Serial.println("Short press");
        }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
}