#pragma once

#include <string>

template <typename T>
class Sensor
{
private:
    std::string m_date;
    std::string m_time;
    std::string m_boardName;
    T m_value, m_min, m_max;

public:
    // constructor sets the date to "XXXXXXXX" to ensure the min/max values are set on the first call to SetValue()
    Sensor() : m_date("XXXXXXXX"), m_time("XX:XX"), m_boardName("X") {};

    bool SetValue(T value, std::string boardName);
    std::string GetName() { return m_boardName; }
    T GetValue() { return m_value; }
    T GetMin() { return m_min; }
    T GetMax() { return m_max; }
    std::string GetTime() { return m_time; }
};

template <typename T>
bool Sensor<T>::SetValue(T value, std::string boardName)
{
    struct tm timeinfo;
    char date_str[9]; // YYYYmmdd
    char time_str[6]; // HH:MM

    m_value = value;
    m_boardName = boardName;

    if (value > m_max)
    {
        m_max = value;
    }
    if (value < m_min)
    {
        m_min = value;
    }

    //
    // Update the time and reset the min/max values if the date has changed
    // * this is a much faster way to determine if the time has been previously sycnhronised\
    // todo: consider adding a max ms time to the getLocalTime(..., maxms) call
    if (time(nullptr) < 100000)
    {
        Serial.println("Time has not been synchronised");
        return false;
    }
    else
    {
        if (!getLocalTime(&timeinfo))
        {
            Serial.println("Failed to obtain local time");
            return false;
        }
    }

    strftime(date_str, 9, "%Y%m%d", &timeinfo); // 4 digit year, 2 digit month, 2 digit day + NULL
    Serial.print("current date: ");
    Serial.println(date_str);

    strftime(time_str, 6, "%H:%M", &timeinfo); // 2 digit hour + ':' + 2 digit minute + NULL
    Serial.print("current time: ");
    Serial.println(time_str);
    m_time = time_str;

    // Note that the date is initialised to "XXXXXXXX", so it won't match the actual date on first receipt of data
    // This ensures that the m_min and m_max values are initialised to the first reading.
    if (date_str != m_date)
    {
        m_date = date_str;
        m_min = value;
        m_max = value;
    }

    return true;
}
