#include <Arduino.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <WiFi.h>
#include <time.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ezButton.h>

#include "DFRobot_RGBLCD1602.h"

// The pin the button is wired to
#define SWITCH_PIN 23

// How many ms equates to a long button press
#define LONG_PRESS_MS 5000

// NTP server details
const char *ntpServer = "pool.ntp.org";
long gmtOffset_sec = 9 * 3600 + 30 * 60; // Default Adelaide timezone (UTC+9:30)
const int daylightOffset_sec = 3600;     // Daylight saving time offset (1 hour)

// Custom parameter for time offset
char timeOffset[6] = "9.5"; // Default value

// LCD display is 16 characters and 2 lines
DFRobot_RGBLCD1602 lcd(16, 2);

ezButton button(SWITCH_PIN); // create ezButton object that attaches to pin SWITCH_PIN;

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
    // wm.resetSettings();

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ("AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid

    res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    // res = wm.autoConnect("AutoConnectAP", "password"); // password protected ap

    if (!res)
    {
        Serial.println("Failed to connect");
        // Uncomment to restart the ESP device if connection fails
        ESP.restart();
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
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    Serial.println("Waiting for NTP time sync...");
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
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

void listSPIFFSFiles()
{
    Serial.println("Listing SPIFFS files:");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file)
    {
        Serial.print("FILE: ");
        Serial.print(file.name());
        Serial.print("\tSIZE: ");
        Serial.println(file.size());
        file = root.openNextFile();
    }
}

void setup()
{
    // Initialize Serial Monitor
    Serial.begin(115200);
    delay(2000); // Delay to allow the serial monitor to initialize

    pinMode(SWITCH_PIN, INPUT_PULLUP);
    button.setDebounceTime(50); // set debounce time to 50 milliseconds

    // initialize the DFRobot LCD
    lcd.init();
    // lcd.clear();
    lcd.print("WiFi...");

    // Initialize SPIFFS
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    // List SPIFFS files
    listSPIFFSFiles();

    // Load the time offset from the file
    loadTimeOffset();

    setupWiFi();
    setupNTP();
}

void loop()
{
    static unsigned long buttonPressedTime;

    // put your main code here, to run repeatedly:
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
        Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
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

        delay(10000); // Print the time every 10 seconds
    }
}