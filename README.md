# ESP32-Standalone-Indoor-Weather-Station

A weather station that includes:

ESP32 devkit or similar 30 pin device;
DFRobot 1602 LCD display;
Adafruit BME280 module that provides temperature, humidity and air pressure;
A momentary on pushbutton.

It was developed using MS Visual Studio Code and PlatformIO.

The device uses a WiFi Manager to connect to a user selectable WiFi AP for the sole purpose of synchronising the time using NTP. This allows the device to display the minimum and maximum values for each sensor for the current day. The device will operate without a WiFi connection but the min and max values will be since the device was powered on. They can only be cleared by cycling the power.

The file "Standalone Weather Station.pdf" contains a wiring schematic.

The following libraries are required:

DFRobot_RBGLCD1602
tzapu/WiFiManager@^2.0.17
arduinogetstarted/ezButton@^1.0.4
adafruit/Adafruit BME280 Library@^2.2.4
adafruit/Adafruit Unified Sensor @ ^1.1.14