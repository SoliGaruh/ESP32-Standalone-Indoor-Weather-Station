#include <Arduino.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <WiFi.h>
#include <time.h>
#include <FS.h>
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
#define LONG_PRESS_MS 2000

// todo: set these
// #define BME_SCK 13
// #define BME_MISO 12
// #define BME_MOSI 11
// #define BME_CS 10

#define BME_SCK 22
#define BME_MISO 12
#define BME_MOSI 21
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C sensor

// LCD display is 16 characters and 2 lines. RGB address 0x6B is for LCD1602 v1.1
DFRobot_RGBLCD1602 lcd(0x6B, 16, 2);

ezButton button(BUTTON_PIN); // create ezButton object that attaches to pin BUTTON_PIN;

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

enum status : char
{
    ok = 0,
    failed
};

// RTOS queue settings

// Size of msg_queue. Needs to be large enough to hold all messages.
// Entering WiFi credentials can cause up to 6 messages to be generated
const int msg_queue_len = 10;
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

void timeSync(const char *tzInfo, const char *ntpServer1, const char *ntpServer2)
{
    // Accurate time is necessary for certificate validion

    configTzTime(tzInfo, ntpServer1, ntpServer2);

    // Wait till time is synced
    Serial.print("Syncing time");
    int i = 0;
    while (time(nullptr) < 1000000000l && i < 40)
    {
        Serial.print(".");
        delay(500);
        i++;
    }
    Serial.println();

    // Show time
    time_t tnow = time(nullptr);
    Serial.print("Synchronized time: ");
    Serial.println(ctime(&tnow));
}

void setupWiFi(int timeoutSeconds, bool resetSettings = false)
{
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

    // WiFiManager, Local initialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;

    if (resetSettings)
    {
        wm.resetSettings();
    }

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ("AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result
    //
    // The device still works if the WiFi is disconnected, but the time will not be updated

    bool res;

    wm.setTimeout(timeoutSeconds);         // Set timeout in seconds
    res = wm.autoConnect("AutoConnectAP"); // anonymous ap

    if (!res)
    {
        Serial.println("Failed to connect");
    }
    else
    {
        Serial.println("Connected to WiFi");
    }
}

void readSensors(void *parameter)
{
    message_t msg;
    unsigned readingId = 0;

    while (1)
    {
        // Read the sensors
        msg.temperature = bme.readTemperature();
        msg.tIsValid = (msg.temperature != NAN);

        msg.pressure = static_cast<unsigned>(bme.readPressure() / 100.0F);
        msg.pIsValid = (msg.pressure != 0);

        msg.humidity = static_cast<unsigned>(bme.readHumidity());
        msg.hIsValid = (msg.humidity != 0);

        msg.readingId = readingId++;

        xQueueSend(msg_queue, (void *)&msg, 10 / portTICK_PERIOD_MS);

        // when the wifi is disconnected the main loop takes ages to process the queue, so the task needs to wait
        // or risk overflowing the queue
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

void DisplayTemperature(Sensor<float> &ts)
{
    lcd.clear();
    //    lcd.printf("Temp:%3.1f %s", ts.GetValue(), ts.GetTime().c_str());
    lcd.printf("Temp:%3.1f", ts.GetValue());
    lcd.setCursor(11, 0);
    lcd.print(ts.GetTime().c_str());

    lcd.setCursor(0, 1);
    lcd.printf("L:%3.1f H:%3.1f",
               ts.GetMin(),
               ts.GetMax());
}

void DisplayPressure(Sensor<unsigned> &ps)
{
    lcd.clear();
    //    lcd.printf("Press:%4d %s", ps.GetValue(), ps.GetTime().c_str());
    lcd.printf("Press:%4d", ps.GetValue());
    lcd.setCursor(11, 0);
    lcd.print(ps.GetTime().c_str());

    lcd.setCursor(0, 1);
    lcd.printf("L:%4d H:%4d",
               ps.GetMin(),
               ps.GetMax());
}

void DisplayHumidity(Sensor<unsigned> &hs)
{
    lcd.clear();
    //    lcd.printf("Hum:%2d %s", hs.GetValue(), hs.GetTime().c_str());
    lcd.printf("Hum:%2d", hs.GetValue());
    lcd.setCursor(11, 0);
    lcd.print(hs.GetTime().c_str());

    lcd.setCursor(0, 1);
    lcd.printf("L:%2d H:%2d",
               hs.GetMin(),
               hs.GetMax());
}

void setup()
{
    // Initialize Serial Monitor
    Serial.begin(115200);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    button.setDebounceTime(50); // set debounce time to 50 milliseconds

    bool status = bme.begin();
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status)
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x");
        Serial.println(bme.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
    }

    // initialize the LCD
    lcd.init();
    lcd.print("Initialising...");

    // setup the wifi but using a short timeout and don't reset AP values. That way, the WiFi will connect
    // if it has been connected before. The user must long press to reset the WiFi credentials.
    setupWiFi(5);

    if (WiFi.status() == WL_CONNECTED)
    {
        timeSync("ACST-9:30ACDT,M10.1.0,M4.1.0/3", "pool.ntp.org", "time.nis.gov");

        // the time has been synced so we don't need the wifi anymore
        // it seems to be a bad idea to turn off the wifi radio, so just disconnect
        WiFi.disconnect();
    }

    // Create the message queue
    msg_queue = xQueueCreate(msg_queue_len, sizeof(message_t));

    // this function is specific to ESP32 RTOS with core parameter added
    xTaskCreatePinnedToCore(
        readSensors,    // Function that should be called
        "Read Sensors", // Name of the task (for debugging)
        4096,           // Stack size (bytes)
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

        // refresh the display.
        if (currentDisplayMode == TemperatureMode)
        {
            DisplayTemperature(temperatureSensor);
        }
        else if (currentDisplayMode == PressureMode)
        {
            DisplayPressure(pressureSensor);
        }
        else if (currentDisplayMode == HumidityMode)
        {
            DisplayHumidity(humiditySensor);
        }
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

            // setup the wifi
            lcd.clear();
            lcd.print("Setup WiFi...");

            setupWiFi(300, true); // timeout is 5 minutes and AP values are cleared

            if (WiFi.status() == WL_CONNECTED)
            {
                timeSync("ACST-9:30ACDT,M10.1.0,M4.1.0/3", "pool.ntp.org", "time.nis.gov");

                // the time has been synced so we don't need the wifi anymore
                WiFi.disconnect();
            }
        }
        else
        {
            // short press signifies the user wants to change mode, ie, from Temperature to Pressure
            Serial.println("Short press");

            // this will cycle through the enum values. Note that HumidityMode MUST be the last
            // value defined in the displayMode enumerated type for this to work.
            currentDisplayMode = (enum displayMode)((currentDisplayMode + 1) % (HumidityMode + 1));
        }
        // refresh the display.
        if (currentDisplayMode == TemperatureMode)
        {
            DisplayTemperature(temperatureSensor);
        }
        else if (currentDisplayMode == PressureMode)
        {
            DisplayPressure(pressureSensor);
        }
        else if (currentDisplayMode == HumidityMode)
        {
            DisplayHumidity(humiditySensor);
        }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
}