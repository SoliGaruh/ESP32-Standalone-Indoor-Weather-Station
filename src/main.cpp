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

// Define the pin for the photoresistor. Note that ADC2 pins cannot be used when WiFi is being used.
// This is because the WiFi driver uses ADC2 pins. The photoresistor is connected to an ADC1 pin.
#define photoResistorPin 34

// How many ms equates to a long button press
#define LONG_PRESS_MS 2000

#define BME_SCK 22
#define BME_MISO 12
#define BME_MOSI 21
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C sensor

// Create a mutex handle to ensure the LCD is not accessed by multiple tasks at the same time
SemaphoreHandle_t xLCDMutex;

// LCD display is 16 characters and 2 lines. RGB address 0x60 is for LCD1602 v1.0
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
// Entering WiFi credentials can cause a message a minute to be generated during the 5 minute timeout.
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

void adjustBacklight(void *parameter)
{
    int photoValue;
    int pwm;

    while (1)
    {
        // Read the value from the photo resistor. the value will be between 0 and 4095. the higher the value, the brighter the light
        photoValue = analogRead(photoResistorPin);

        Serial.print("Photo Resistor Value: ");
        Serial.println(photoValue); // Print the value to the serial monitor

        pwm = map(photoValue, 0, 2000, 125, 255);
        if (pwm > 255)
        {
            pwm = 255;
        }

        // Protect access to the LCD with the mutex
        if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdTRUE)
        {
            // Set the backlight intensity to the mapped value
            lcd.setPWM(lcd.REG_ONLY, pwm);

            // Release the mutex
            xSemaphoreGive(xLCDMutex);
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
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

        // The sensors are read once per minute
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

void DisplayTemperature(Sensor<float> &ts)
{
    // Protect access to the LCD with the mutex
    if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdTRUE)
    {
        lcd.clear();
        lcd.printf("Temp:%3.1f", ts.GetValue());
        lcd.setCursor(11, 0);
        lcd.print(ts.GetTime().c_str());
        lcd.setCursor(0, 1);
        lcd.printf("L:%3.1f H:%3.1f",
                   ts.GetMin(),
                   ts.GetMax());

        // Release the mutex
        xSemaphoreGive(xLCDMutex);
    }
}

void DisplayPressure(Sensor<unsigned> &ps)
{
    // Protect access to the LCD with the mutex
    if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdTRUE)
    {
        lcd.clear();
        lcd.printf("Press:%4d", ps.GetValue());
        lcd.setCursor(11, 0);
        lcd.print(ps.GetTime().c_str());
        lcd.setCursor(0, 1);
        lcd.printf("L:%4d H:%4d",
                   ps.GetMin(),
                   ps.GetMax());

        // Release the mutex
        xSemaphoreGive(xLCDMutex);
    }
}

void DisplayHumidity(Sensor<unsigned> &hs)
{
    // Protect access to the LCD with the mutex
    if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdTRUE)
    {
        lcd.clear();
        lcd.printf("Hum:%2d", hs.GetValue());
        lcd.setCursor(11, 0);
        lcd.print(hs.GetTime().c_str());
        lcd.setCursor(0, 1);
        lcd.printf("L:%2d H:%2d",
                   hs.GetMin(),
                   hs.GetMax());

        // Release the mutex
        xSemaphoreGive(xLCDMutex);
    }
}

void setup()
{
    // Initialize Serial Monitor
    Serial.begin(115200);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    pinMode(photoResistorPin, INPUT); // Set the photo resistor pin as an input
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    button.setDebounceTime(50); // set debounce time to 50 milliseconds

    bool status = bme.begin();
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

    // Create the mutex for LCD access
    xLCDMutex = xSemaphoreCreateMutex();

    // initialize the LCD. Don't need to protect this with the mutex as it is only called once. There are
    // no other tasks running at this point.
    lcd.init();
    lcd.print("Initialising...");

    // setup the wifi but using a short timeout and don't reset AP values. That way, the WiFi will connect
    // if it has been connected before. The user must long press to reset the WiFi credentials.
    setupWiFi(5);

    if (WiFi.status() == WL_CONNECTED)
    {
        timeSync("ACST-9:30ACDT,M10.1.0,M4.1.0/3", "pool.ntp.org", "time.nis.gov");
    }

    // disconnect the WiFi as it is no longer needed. Only used to grab the time. Turn off the WiFi radio as well to generate
    // less heat.

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    // Create the message queue used to pass sensor data to the main loop
    msg_queue = xQueueCreate(msg_queue_len, sizeof(message_t));

    // Define the task that will read the sensors
    xTaskCreatePinnedToCore(
        readSensors,    // Function that should be called
        "Read Sensors", // Name of the task (for debugging)
        4096,           // Stack size (bytes)
        NULL,           // Parameter to pass
        1,              // Task priority
        NULL,           // Task handle
        app_cpu);       // which core to use

    // Define the task that will adjust the backlight
    xTaskCreatePinnedToCore(
        adjustBacklight,    // Function that should be called
        "Adjust Backlight", // Name of the task (for debugging)
        2048,               // Stack size (bytes)
        NULL,               // Parameter to pass
        1,                  // Task priority
        NULL,               // Task handle
        app_cpu);           // which core to use
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
        else
        {
            Serial.println("Temperature reading is invalid");
        }

        if (incomingReadings.pIsValid)
        {
            Serial.printf("Pressure: %d\n", incomingReadings.pressure);
            pressureSensor.SetValue(incomingReadings.pressure, "Builtin");
        }
        else
        {
            Serial.println("Pressure reading is invalid");
        }

        if (incomingReadings.hIsValid)
        {
            Serial.printf("Humidity: %d\n", incomingReadings.humidity);
            humiditySensor.SetValue(incomingReadings.humidity, "Builtin");
        }
        else
        {
            Serial.println("Humidity reading is invalid");
        }

        Serial.printf("Reading ID: %d\n\n", incomingReadings.readingId);

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

            // Protect access to the LCD with the mutex
            if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdTRUE)
            {
                lcd.clear();
                lcd.print("Setup WiFi...");

                // Release the mutex
                xSemaphoreGive(xLCDMutex);
            }

            // setup the wifi
            setupWiFi(300, true); // timeout is 5 minutes and AP values are cleared

            if (WiFi.status() == WL_CONNECTED)
            {
                timeSync("ACST-9:30ACDT,M10.1.0,M4.1.0/3", "pool.ntp.org", "time.nis.gov");
            }

            // disconnect the WiFi as it is no longer needed. Only used to grab the time. Turn off the WiFi radio as well to generate
            // less heat.
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
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
    vTaskDelay(100 / portTICK_PERIOD_MS);
}