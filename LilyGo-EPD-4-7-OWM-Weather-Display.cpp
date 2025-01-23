// ESP32 Weather Display and a LilyGo EPD 4.7" Display, obtains Open Weather Map data, decodes and then displays it.
// This software, the ideas and concepts is Copyright (c) David Bird 2021. All rights to this software are reserved.
// #################################################################################################################

#include <Arduino.h>           // In-built
#include "esp_task_wdt.h"      // In-built
#include "freertos/FreeRTOS.h" // In-built
#include "freertos/task.h"     // In-built
#include "epd_driver.h"        // https://github.com/Xinyuan-LilyGO/LilyGo-EPD47
#include "esp_adc_cal.h"       // In-built
#include "driver/uart.h"       // In-built

#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <HTTPClient.h>  // In-built

#include <WiFi.h> // In-built
#include <SPI.h>  // In-built
#include <time.h> // In-built
#include "SPIFFS.h"
#include "FS.h"

#define FILENAME "/data.csv" // File to store sensor data

// Sensor reading tasks
#include "bmp280.h"
#include "sht4x.h"
#include "esp_now.h"
#include "esp_system.h"
#include "esp_err.h"


#include "owm_credentials.h"
#include "forecast_record.h"
#include "web.h"
#include "LilyGo-EPD-4-7-OWM-Weather-Display.h"
#include "drawingFunctions.h"

#if T5_47_PLUS_V2
#define USR_BUTTON GPIO_NUM_21
#define I2C_MASTER_SDA GPIO_NUM_17
#define I2C_MASTER_SCL GPIO_NUM_18
#elif CONFIG_IDF_TARGET_ESP32
#define USR_BUTTON GPIO_NUM_21
#elif CONFIG_IDF_TARGET_ESP32S3
#define USR_BUTTON GPIO_NUM_21
#endif

#define SCREEN_WIDTH EPD_WIDTH
#define SCREEN_HEIGHT EPD_HEIGHT
#define MINUTES_TO_TICKS(minutes) pdMS_TO_TICKS((minutes) * 60 * 1000) // converting minutes to seconds and then miliseconds

//################  VERSION  ##################################################
String version = "2.5 / 4.7in"; // Programme version, see change log at end
//################ VARIABLES ##################################################


//################ PROGRAM VARIABLES and OBJECTS ##########################################

String Time_str = "--:--:--";
String Date_str = "-- --- ----";

#define max_readings 8 // (was 24) Limited to 3-days here, but could go to 5-days = 40

volatile int screenState = 0; // default screen state

Forecast_record_type WxConditions[1];
Forecast_record_type WxForecast[max_readings];

float pressure_readings[max_readings]    = {0};
float temperature_readings[max_readings] = {0};
float humidity_readings[max_readings]    = {0};
float rain_readings[max_readings]        = {0};
float snow_readings[max_readings]        = {0};

const int SleepDuration = 10; // Sleep time in minutes, aligned to the nearest minute boundary, so if 30 will always update at 00 or 30 past the hour
bool SleepHoursEnabled = false;
bool DeepSleepEnabled = false;
int WakeupHour    = 5;  // Don't wakeup until after 07:00 to save battery power
int SleepHour     = 1; // Sleep after 23:00 to save battery power
long StartTime     = 0;
long SleepTimer    = 0;
long Delta         = 30; // ESP32 rtc speed compensation, prevents display at xx:59:yy and then xx:00:yy (one minute later) to save power

// Semaphore handles
SemaphoreHandle_t configSemaphore;
SemaphoreHandle_t SHT4XTriggerSem;
SemaphoreHandle_t BME280TriggerSem;
SemaphoreHandle_t sensorDataReadySem;
SemaphoreHandle_t dataProcessedSem;
SemaphoreHandle_t historyCalcMutex;
SemaphoreHandle_t i2cMutex;

// Queue handles
QueueHandle_t sensorDataQueue;
QueueHandle_t processedDataQueue;

// Define structure for sensor data
typedef struct {
    float temperature;
    float humidity;
    float pressure;
} UntaggedSensorData;

typedef enum {
    SENSOR_NONE = 0,
    SENSOR_BME280,
    SENSOR_SHT4X
} SensorType;

typedef struct {
    SensorType tag;
    float temperature;
    float humidity;
    float pressure;
} TaggedSensorData;

// History for data bias calculation
int historyIndex = 0;
int numReadings = 0;
UntaggedSensorData readingHistory[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
UntaggedSensorData processedResult;


bool wakeInterruptFlag = false;

//################ ISR ##################################################

// Button interrupt handler
volatile bool ButtonPressed = false; // Flag to signal the main loop
volatile unsigned long lastPressTime = 0; // Last time the button was pressed

void IRAM_ATTR handleButtonPress()
{
    const unsigned long debounceDelay = 200; // Debounce delay in milliseconds
    unsigned long currentTime = millis();
    // Check for debounce and state
    if ((currentTime - lastPressTime) > debounceDelay) {
        // flag
        ButtonPressed = true;
        lastPressTime = currentTime;
    }
}

//################ FUNCTIONS ##################################################

// Clears screen buffer
void InitialiseDisplay()
{
    epd_init();
    framebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), EPD_WIDTH * EPD_HEIGHT / 2);
    if (!framebuffer)
        Serial.println("Memory alloc failed!");
    memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
}

void InitialiseSystem()
{
    StartTime = millis();
    Serial.begin(115200);

    unsigned long serialTimeout = millis(); // if serial hasn't started waiting 2000 ms then continuing 
    while (!Serial && (millis() - serialTimeout < 3000)){
        delay(200);
        Serial.print(".");  // Show progress
    }
    Serial.println(String(__FILE__) + "\nInitializing System...");

    ESP_ERROR_CHECK(i2cdev_init()); // Initialize the I2C bus
    delay(1000);
    InitialiseDisplay();
}

void InitiateSleep()
{
    epd_poweroff_all();
    UpdateLocalTime();

    // If SleepHoursEnabled and it's past SleepHour or before WakeupHour, sleep until defined WakeupHour
    if (SleepHoursEnabled == true && (CurrentHour >= SleepHour || CurrentHour < WakeupHour))
    {   
        int currentMinutes = CurrentHour * 60 + CurrentMin; // Current time in minutes
        int wakeupMinutes = WakeupHour * 60;                // Wake-up time in minutes

        // If it's past SleepHour or before WakeupHour, sleep until WakeupHour
        SleepTimer = (wakeupMinutes - currentMinutes + 1440) % 1440 * 60; // Time until wakeup (in seconds)
        Serial.println("Sleeping until WakeupHour " + String(WakeupHour) + ":00");
    }
    else // Otherwise, sleep for the regular duration defined by SleepDuration
    {
        SleepTimer = (SleepDuration * 60 - ((CurrentMin % SleepDuration) * 60 + CurrentSec)) + Delta; //Some ESP32 have a RTC that is too fast to maintain accurate time, so add an offset
        Serial.println("Sleeping for " + String(SleepDuration) + " minutes");
    }
    
    // Set wakeup timer (1000000LL converts to Secs as unit = 1uSec)
    esp_sleep_enable_timer_wakeup(SleepTimer * 1000000LL); 
    Serial.println("Awake for: " + String((millis() - StartTime) / 1000.0, 3) + " secs");
    Serial.println("Entering " + String(SleepTimer) + " (secs) of sleep time");

    //Select deep or light sleep
    if(DeepSleepEnabled == true)
    {
        Serial.println("Starting deep sleep period");
        esp_deep_sleep_start();
    }
    else
    {
        //esp_sleep_enable_uart_wakeup(UART_NUM_0);
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); // Keep peripherals powered
        //esp_sleep_enable_ext0_wakeup(USR_BUTTON, 1); // Wake up on button press

        ESP_LOGI("SLEEP", "Starting light sleep period");
        Serial.end();           // Close existing serial connection
        esp_light_sleep_start(); // Sleep for SleepDuration minutes

        InitialiseSystem();
        ESP_LOGI("SLEEP", "Woke up from light sleep.");
    }
}

boolean SetTime()
{
    configTime(gmtOffset_sec, daylightOffset_sec, const_cast<const char *>(ntpServer.c_str()), "time.nist.gov"); //(gmtOffset_sec, daylightOffset_sec, ntpServer)
    setenv("TZ", const_cast<const char *>(Timezone.c_str()), 1);                                                 //setenv()adds the "TZ" variable to the environment with a value TimeZone, only used if set to 1, 0 means no change
    tzset();                                                                   // Set the TZ environment variable
    delay(100);
    return UpdateLocalTime();
}

uint8_t StartWiFi()
{
    Serial.println("\r\nConnecting to: " + String(ssid));
    IPAddress dns(8, 8, 8, 8); // Use Google DNS
    WiFi.disconnect();
    WiFi.mode(WIFI_STA); // switch off AP
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.begin(const_cast<const char *>(ssid.c_str()), const_cast<const char *>(password.c_str()));
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.printf("STA: Failed!\n");
        WiFi.disconnect(false);
        delay(500);
        WiFi.begin(const_cast<const char *>(ssid.c_str()), const_cast<const char *>(password.c_str()));
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        wifi_signal = WiFi.RSSI(); // Get Wifi Signal strength now, because the WiFi will be turned off to save power!
        Serial.println("WiFi connected at: " + WiFi.localIP().toString());
    }
    else
        Serial.println("WiFi connection *** FAILED ***");
    return WiFi.status();
}

void StopWiFi()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi switched Off");
}

bool DecodeWeather(WiFiClient &json, String Type)
{
    Serial.print(F("\nCreating object...and "));
    DynamicJsonDocument doc(64 * 1024);                      // allocate the JsonDocument
    DeserializationError error = deserializeJson(doc, json); // Deserialize the JSON document
    if (error)
    { // Test if parsing succeeds.
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return false;
    }
    // convert it to a JsonObject
    JsonObject root = doc.as<JsonObject>();
    Serial.println(" Decoding " + Type + " data");
    if (Type == "weather")
    {
        // All Serial.println statements are for diagnostic purposes and some are not required, remove if not needed with //
        //WxConditions[0].lon         = root["coord"]["lon"].as<float>();              Serial.println(" Lon: " + String(WxConditions[0].lon));
        //WxConditions[0].lat         = root["coord"]["lat"].as<float>();              Serial.println(" Lat: " + String(WxConditions[0].lat));
        WxConditions[0].Main0 = root["weather"][0]["main"].as<const char *>();
        Serial.println("Main: " + String(WxConditions[0].Main0));
        WxConditions[0].Forecast0 = root["weather"][0]["description"].as<const char *>();
        Serial.println("For0: " + String(WxConditions[0].Forecast0));
        //WxConditions[0].Forecast1   = root["weather"][1]["description"].as<char*>(); Serial.println("For1: " + String(WxConditions[0].Forecast1));
        //WxConditions[0].Forecast2   = root["weather"][2]["description"].as<char*>(); Serial.println("For2: " + String(WxConditions[0].Forecast2));
        WxConditions[0].Icon = root["weather"][0]["icon"].as<const char *>();
        Serial.println("Icon: " + String(WxConditions[0].Icon));
        WxConditions[0].Temperature = root["main"]["temp"].as<float>();
        Serial.println("Temp: " + String(WxConditions[0].Temperature));
        WxConditions[0].Pressure = root["main"]["pressure"].as<float>();
        Serial.println("Pres: " + String(WxConditions[0].Pressure));
        WxConditions[0].Humidity = root["main"]["humidity"].as<float>();
        Serial.println("Humi: " + String(WxConditions[0].Humidity));
        WxConditions[0].Low = root["main"]["temp_min"].as<float>();
        Serial.println("TLow: " + String(WxConditions[0].Low));
        WxConditions[0].High = root["main"]["temp_max"].as<float>();
        Serial.println("THig: " + String(WxConditions[0].High));
        WxConditions[0].Windspeed = root["wind"]["speed"].as<float>();
        Serial.println("WSpd: " + String(WxConditions[0].Windspeed));
        WxConditions[0].Winddir = root["wind"]["deg"].as<float>();
        Serial.println("WDir: " + String(WxConditions[0].Winddir));
        WxConditions[0].Cloudcover = root["clouds"]["all"].as<int>();
        Serial.println("CCov: " + String(WxConditions[0].Cloudcover)); // in % of cloud cover
        WxConditions[0].Visibility = root["visibility"].as<int>();
        Serial.println("Visi: " + String(WxConditions[0].Visibility)); // in metres
        WxConditions[0].Rainfall = root["rain"]["1h"].as<float>();
        Serial.println("Rain: " + String(WxConditions[0].Rainfall));
        WxConditions[0].Snowfall = root["snow"]["1h"].as<float>();
        Serial.println("Snow: " + String(WxConditions[0].Snowfall));
        //WxConditions[0].Country     = root["sys"]["country"].as<char*>();            Serial.println("Ctry: " + String(WxConditions[0].Country));
        WxConditions[0].Sunrise = root["sys"]["sunrise"].as<int>();
        Serial.println("SRis: " + String(WxConditions[0].Sunrise));
        WxConditions[0].Sunset = root["sys"]["sunset"].as<int>();
        Serial.println("SSet: " + String(WxConditions[0].Sunset));
        WxConditions[0].Timezone = root["timezone"].as<int>();
        Serial.println("TZon: " + String(WxConditions[0].Timezone));
    }
    if (Type == "forecast")
    {
        //Serial.println(json);
        Serial.print(F("\nReceiving Forecast period - ")); //------------------------------------------------
        JsonArray list = root["list"];
        for (byte r = 0; r < max_readings; r++)
        {
            Serial.println("\nPeriod-" + String(r) + "--------------");
            WxForecast[r].Dt = list[r]["dt"].as<int>();
            WxForecast[r].Temperature = list[r]["main"]["temp"].as<float>();
            Serial.println("Temp: " + String(WxForecast[r].Temperature));
            WxForecast[r].Low = list[r]["main"]["temp_min"].as<float>();
            Serial.println("TLow: " + String(WxForecast[r].Low));
            WxForecast[r].High = list[r]["main"]["temp_max"].as<float>();
            Serial.println("THig: " + String(WxForecast[r].High));
            WxForecast[r].Pressure = list[r]["main"]["pressure"].as<float>();
            Serial.println("Pres: " + String(WxForecast[r].Pressure));
            WxForecast[r].Humidity = list[r]["main"]["humidity"].as<float>();
            Serial.println("Humi: " + String(WxForecast[r].Humidity));
            //WxForecast[r].Forecast0         = list[r]["weather"][0]["main"].as<char*>();        Serial.println("For0: " + String(WxForecast[r].Forecast0));
            //WxForecast[r].Forecast1         = list[r]["weather"][1]["main"].as<char*>();        Serial.println("For1: " + String(WxForecast[r].Forecast1));
            //WxForecast[r].Forecast2         = list[r]["weather"][2]["main"].as<char*>();        Serial.println("For2: " + String(WxForecast[r].Forecast2));
            WxForecast[r].Icon = list[r]["weather"][0]["icon"].as<const char *>();
            Serial.println("Icon: " + String(WxForecast[r].Icon));
            //WxForecast[r].Description       = list[r]["weather"][0]["description"].as<char*>(); Serial.println("Desc: " + String(WxForecast[r].Description));
            //WxForecast[r].Cloudcover        = list[r]["clouds"]["all"].as<int>();               Serial.println("CCov: " + String(WxForecast[r].Cloudcover)); // in % of cloud cover
            //WxForecast[r].Windspeed         = list[r]["wind"]["speed"].as<float>();             Serial.println("WSpd: " + String(WxForecast[r].Windspeed));
            //WxForecast[r].Winddir           = list[r]["wind"]["deg"].as<float>();               Serial.println("WDir: " + String(WxForecast[r].Winddir));
            WxForecast[r].Rainfall = list[r]["rain"]["3h"].as<float>();
            Serial.println("Rain: " + String(WxForecast[r].Rainfall));
            WxForecast[r].Snowfall = list[r]["snow"]["3h"].as<float>();
            Serial.println("Snow: " + String(WxForecast[r].Snowfall));
            WxForecast[r].Period = list[r]["dt_txt"].as<const char *>();
            Serial.println("Peri: " + String(WxForecast[r].Period));
        }
        //------------------------------------------
        float pressure_trend = WxForecast[0].Pressure - WxForecast[2].Pressure; // Measure pressure slope between ~now and later
        pressure_trend = ((int)(pressure_trend * 10)) / 10.0;                   // Remove any small variations less than 0.1
        WxConditions[0].Trend = "=";
        if (pressure_trend > 0)
            WxConditions[0].Trend = "+";
        if (pressure_trend < 0)
            WxConditions[0].Trend = "-";
        if (pressure_trend == 0)
            WxConditions[0].Trend = "0";

        if (Units == "I")
            Convert_Readings_to_Imperial();
    }
    return true;
}

bool obtainWeatherData(WiFiClient &client, const String &RequestType)
{
    const String units = (Units == "M" ? "metric" : "imperial");
    client.stop(); // close connection before sending a new request
    HTTPClient http;
    String uri = "/data/2.5/" + RequestType + "?q=" + City + "," + Country + "&APPID=" + apikey + "&mode=json&units=" + units + "&lang=" + Language;
    if (RequestType != "weather")
    {
        uri += "&cnt=" + String(max_readings);
    }
    http.begin(client, server, 80, uri); //http.begin(uri,test_root_ca); //HTTPS example connection
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK)
    {
        if (!DecodeWeather(http.getStream(), RequestType))
            return false;
        client.stop();
        http.end();
        return true;
    }
    else
    {
        Serial.printf("connection failed, error: %s", http.errorToString(httpCode).c_str());
        client.stop();
        http.end();
        return false;
    }
    http.end();
    return true;
}

float SumOfPrecip(float DataArray[], int readings)
{
    float sum = 0;
    for (int i = 0; i <= readings; i++)
    {
        sum += DataArray[i];
    }
    return sum;
}

String TitleCase(String text)
{
    if (text.length() > 0)
    {
        String temp_text = text.substring(0, 1);
        temp_text.toUpperCase();
        return temp_text + text.substring(1); // Title-case the string
    }
    else
        return text;
}

void DisplayWeather_Screen0()
{   // 4.7" e-paper display is 960x540 resolution
    DisplayStatusSection(600, 20, wifi_signal); // Wi-Fi signal strength and Battery voltage
    DisplayGeneralInfoSection();                // Top line of the display

    DisplayDisplayWindSection(130, 140, WxConditions[0].Winddir, WxConditions[0].Windspeed*3.6, 100); // Converting wind speed from m/s to km/h for display
    DisplayAstronomySection(5, 250);     // Astronomy section Sun rise/set, Moon phase and Moon icon

    DisplayMainWeatherSection(250, 120); // Centre section of display for Location, temperature, Weather report, current Wx Symbol
    DisplaySensorReadings(490, 65);     // Local sensor readings section
    DisplayWeatherIcon(383, 290);        // Display weather icon    scale = Large

    DisplayForecastSection(0, 370);    // 3hr forecast boxes (was 320x220)
}

void DisplayWeather_Screen1()
{   // 4.7" e-paper display is 960x540 resolution
    DisplayStatusSection(600, 20, wifi_signal); // Wi-Fi signal strength and Battery voltage
    DisplayGeneralInfoSection();                // Top line of the display
}

void DisplayWeather_Screen2()
{   // 4.7" e-paper display is 960x540 resolution
    DisplayStatusSection(600, 20, wifi_signal); // Wi-Fi signal strength and Battery voltage
    DisplayGeneralInfoSection();                // Top line of the display
    DisplayWeatherIcon(383, 290);                // Display weather icon    scale = Large
}

// Array of function pointers to select different screens for display
void (*screens[])() = {DisplayWeather_Screen0, DisplayWeather_Screen1, DisplayWeather_Screen2};
void DisplayWeather(volatile int &screenState) 
{
    if ((screenState >= 0) && (screenState < (sizeof(screens) / sizeof(screens[0])))) {
        screens[screenState]();
        ESP_LOGI("DISPLAY", "Displaying screen nr: %d", screenState);
    } else {
        ESP_LOGW("DISPLAY", "Invalid screen state: %d. Defaulting to Screen 0.", screenState);
        screens[0]();
    }
}

void DisplayWeatherIcon(int x, int y)
{
    DisplayConditionsSection(x, y, WxConditions[0].Icon, LargeIcon);
}

void DisplayMainWeatherSection(int x, int y)
{
    DisplayTemperatureSection(x + 10, y);
    // DisplayForecastTextSection(x - 55, y + 25); // Turned off text forecast
    DisplayPressureSection(x+10, y + 50, WxConditions[0].Pressure, WxConditions[0].Trend);
}

void DisplayGeneralInfoSection()
{
    setFont(OpenSans12B);
    drawString(5, 5, City, LEFT);
    setFont(OpenSans18B);
    drawString(170, 0, Date_str, LEFT);
    setFont(OpenSans10B);
    drawString(490, 2, "Aktualizacja: " + Time_str, LEFT);
}

void DisplaySensorReadings(int x, int y)
{
    drawLine(480, 10, 480, 500, DarkGrey);
    drawLine(200, 40, 910, 40, DarkGrey);
    DisplaySensorReadingsGarden(x, 40);
    DisplaySensorReadingsRoom(x+245, 40);
    //drawLine(480, 310, 940, 310, DarkGrey);
}

void DisplaySensorReadingsGarden(int x, int y)
{
    setFont(OpenSans12B);
    drawString(x, y, "Czujnik ZEWN.", LEFT);
    setFont(OpenSans24B);
    //drawString(x, y+40, String("12.6") + "° " + "85" + "%", LEFT);
    drawString(x, y+40, String("12.6") + "°", LEFT);
    setFont(OpenSans18B);
    drawString(x+135, y+40, " " + String("85") + "%", LEFT);
    drawString(x, y+90, String("1024") + " hPa", LEFT);
}

void DisplaySensorReadingsRoom(int x, int y)
{
    if (xQueueReceive(processedDataQueue, &processedResult, pdMS_TO_TICKS(500)) == pdTRUE)
    {
        ESP_LOGI("DISPLAY", "Successfully received processed data from queue");
        setFont(OpenSans12B);
        drawString(x, y, "Czujnik DOM", LEFT);
        setFont(OpenSans24B);
        drawString(x, y+40, String(processedResult.temperature, 1) + "°", LEFT);
        setFont(OpenSans18B);
        drawString(x+135, y+40, " " + String(processedResult.humidity, 0) + "%", LEFT);
        drawString(x, y+90, String(processedResult.pressure, 0) + " hPa", LEFT);
    }
    else
    {
        ESP_LOGW("DISPLAY", "Failed to receive processed data from queue");
        setFont(OpenSans12B);
        drawString(x, y, "Czujnik dom", LEFT);
        setFont(OpenSans24B);
        drawString(x, y+40, String("--.-") + "°", LEFT);
        setFont(OpenSans18B);
        drawString(x+135, y+40, " " + String("--") + "%", LEFT);
        drawString(x, y+90, String("----") + " hPa", LEFT);
    }    
}

void DisplayDisplayWindSection(int x, int y, float angle, float windspeed, int Cradius)
{
    angle = fmod((angle + 180), 360); // Ensure the angle points opposite direction and wraps correctly between 0-360°
    arrow(x, y, Cradius - 22, angle, 18+2, 33+2); // Show wind direction on outer circle of width and length
    setFont(OpenSans8B);
    int dxo, dyo, dxi, dyi;
    drawCircle(x, y, Cradius, Black);       // Draw compass circle
    drawCircle(x, y, Cradius + 1, Black);   // Draw compass circle
    drawCircle(x, y, Cradius * 0.7, Black); // Draw compass inner circle
    for (float a = 0; a < 360; a = a + 22.5)
    {
        dxo = Cradius * cos((a - 90) * PI / 180);
        dyo = Cradius * sin((a - 90) * PI / 180);
        if (a == 45)
            drawString(dxo + x + 15, dyo + y - 18, TXT_NE, CENTER);
        if (a == 135)
            drawString(dxo + x + 20, dyo + y - 2, TXT_SE, CENTER);
        if (a == 225)
            drawString(dxo + x - 20, dyo + y - 2, TXT_SW, CENTER);
        if (a == 315)
            drawString(dxo + x - 15, dyo + y - 18, TXT_NW, CENTER);
        dxi = dxo * 0.9;
        dyi = dyo * 0.9;
        drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Black);
        dxo = dxo * 0.7;
        dyo = dyo * 0.7;
        dxi = dxo * 0.9;
        dyi = dyo * 0.9;
        drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Black);
    }
    drawString(x, y - Cradius - 20, TXT_N, CENTER);
    drawString(x, y + Cradius + 10, TXT_S, CENTER);
    drawString(x - Cradius - 15, y - 5, TXT_W, CENTER);
    drawString(x + Cradius + 10, y - 5, TXT_E, CENTER);
    drawString(x + 3, y + 50, String(angle, 0) + "°", CENTER);
    setFont(OpenSans12B);
    drawString(x, y - 50, WindDegToOrdinalDirection(angle), CENTER);
    setFont(OpenSans24B);
    drawString(x + 3, y - 18, String(windspeed, 1), CENTER);
    setFont(OpenSans12B);
    drawString(x, y + 25, (Units == "M" ? "km/h" : "mph"), CENTER); // change from m/s
}

String WindDegToOrdinalDirection(float winddirection)
{
    if (winddirection >= 348.75 || winddirection < 11.25)
        return TXT_N;
    if (winddirection >= 11.25 && winddirection < 33.75)
        return TXT_NNE;
    if (winddirection >= 33.75 && winddirection < 56.25)
        return TXT_NE;
    if (winddirection >= 56.25 && winddirection < 78.75)
        return TXT_ENE;
    if (winddirection >= 78.75 && winddirection < 101.25)
        return TXT_E;
    if (winddirection >= 101.25 && winddirection < 123.75)
        return TXT_ESE;
    if (winddirection >= 123.75 && winddirection < 146.25)
        return TXT_SE;
    if (winddirection >= 146.25 && winddirection < 168.75)
        return TXT_SSE;
    if (winddirection >= 168.75 && winddirection < 191.25)
        return TXT_S;
    if (winddirection >= 191.25 && winddirection < 213.75)
        return TXT_SSW;
    if (winddirection >= 213.75 && winddirection < 236.25)
        return TXT_SW;
    if (winddirection >= 236.25 && winddirection < 258.75)
        return TXT_WSW;
    if (winddirection >= 258.75 && winddirection < 281.25)
        return TXT_W;
    if (winddirection >= 281.25 && winddirection < 303.75)
        return TXT_WNW;
    if (winddirection >= 303.75 && winddirection < 326.25)
        return TXT_NW;
    if (winddirection >= 326.25 && winddirection < 348.75)
        return TXT_NNW;
    return "?";
}

void DisplayTemperatureSection(int x, int y)
{
    setFont(OpenSans18B);
    drawString(x, y - 40, String(WxConditions[0].Temperature, 1) + "°     " + String(WxConditions[0].Humidity, 0) + "%", LEFT);
    setFont(OpenSans12B);
    drawString(x, y, "Max " + String(WxConditions[0].High, 0) + "° | Min " + String(WxConditions[0].Low, 0) + "°", LEFT); // Show forecast high and Low
}

void DisplayForecastTextSection(int x, int y)
{
#define lineWidth 34
    setFont(OpenSans12B);
    //Wx_Description = WxConditions[0].Main0;          // e.g. typically 'Clouds'
    String Wx_Description = WxConditions[0].Forecast0; // e.g. typically 'overcast clouds' ... you choose which
    Wx_Description.replace(".", "");                   // remove any '.'
    int spaceRemaining = 0, p = 0, charCount = 0, Width = lineWidth;
    while (p < Wx_Description.length())
    {
        if (Wx_Description.substring(p, p + 1) == " ")
            spaceRemaining = p;
        if (charCount > Width - 1)
        { // '~' is the end of line marker
            Wx_Description = Wx_Description.substring(0, spaceRemaining) + "~" + Wx_Description.substring(spaceRemaining + 1);
            charCount = 0;
        }
        p++;
        charCount++;
    }
    if (WxForecast[0].Rainfall > 0)
        Wx_Description += " (" + String(WxForecast[0].Rainfall, 1) + String((Units == "M" ? "mm" : "in")) + ")";
    //Wx_Description = wordWrap(Wx_Description, lineWidth);
    String Line1 = Wx_Description.substring(0, Wx_Description.indexOf("~"));
    String Line2 = Wx_Description.substring(Wx_Description.indexOf("~") + 1);
    drawString(x + 30, y + 5, TitleCase(Line1), LEFT);
    if (Line1 != Line2)
        drawString(x + 30, y + 30, Line2, LEFT);
}

void DisplayPressureSection(int x, int y, float pressure, String slope)
{
    setFont(OpenSans12B);
    DrawPressureAndTrend(x - 20, y - 5, pressure, slope);
    /* // Turned off visivility
    if (WxConditions[0].Visibility > 0)
    {
        Visibility(x + 145, y, String(WxConditions[0].Visibility) + "M");
        x += 150; // Draw the text in the same positions if one is zero, otherwise in-line
    }
    */
    // if (WxConditions[0].Cloudcover > 0) // removed if, cloud cover always being drawn
        CloudCover(x + 130, y - 5, WxConditions[0].Cloudcover);
}

void DisplayForecastWeather(int x, int y, int index)
{
    int fwidth = 120; // EPD_WIDTH
    x = x + fwidth * index;
    DisplayConditionsSection(x + fwidth / 2, y + 90, WxForecast[index].Icon, MediumIcon); // changed from SmallIcon 
    drawLine(x+fwidth, y+10, x+fwidth, y + 160, DarkGrey);
    setFont(OpenSans12B);
    drawString(x + fwidth / 2, y + 10, String(ConvertUnixTime(WxForecast[index].Dt + WxConditions[0].Timezone).substring(0, 5)), CENTER);
    drawString(x + fwidth / 2, y + 135, String(WxForecast[index].High, 0) + "°/" + String(WxForecast[index].Low, 0) + "°", CENTER);
}

void DisplayAstronomySection(int x, int y)
{
    setFont(OpenSans12B);
    drawString(x + 5, y + 30, TXT_SUNRISE + ": " + ConvertUnixTime(WxConditions[0].Sunrise).substring(0, 5), LEFT);
    drawString(x + 5, y + 55, TXT_SUNSET + ":  " + ConvertUnixTime(WxConditions[0].Sunset).substring(0, 5), LEFT);
    time_t now = time(NULL);
    struct tm *now_utc = gmtime(&now);
    const int day_utc = now_utc->tm_mday;
    const int month_utc = now_utc->tm_mon + 1;
    const int year_utc = now_utc->tm_year + 1900;
    drawString(x + 5, y + 80, MoonPhase(day_utc, month_utc, year_utc, Hemisphere), LEFT);
    DrawMoon(x + 150, y - 30, day_utc, month_utc, year_utc, Hemisphere);
}

void DisplayForecastSection(int x, int y)
{
    drawLine(20, 365, 940, 365, DarkGrey);
    int f = 0;
    do
    {
        DisplayForecastWeather(x, y, f);
        f++;
    } while (f < max_readings);
    /* // turning off 4 graphs in the main screen

    int r = 0;
    do
    { // Pre-load temporary arrays with with data - because C parses by reference and remember that[1] has already been converted to I units
        if (Units == "I")
            pressure_readings[r] = WxForecast[r].Pressure * 0.02953;
        else
            pressure_readings[r] = WxForecast[r].Pressure;
        if (Units == "I")
            rain_readings[r] = WxForecast[r].Rainfall * 0.0393701;
        else
            rain_readings[r] = WxForecast[r].Rainfall;
        if (Units == "I")
            snow_readings[r] = WxForecast[r].Snowfall * 0.0393701;
        else
            snow_readings[r] = WxForecast[r].Snowfall;
        temperature_readings[r] = WxForecast[r].Temperature;
        humidity_readings[r] = WxForecast[r].Humidity;
        r++;
    } while (r < max_readings);

    int gwidth = 175, gheight = 100;
    int gx = (SCREEN_WIDTH - gwidth * 4) / 5 + 8; // equals 60px
    int gy = (SCREEN_HEIGHT - gheight - 30); // equals 410
    int gap = gwidth + gx;

    // (x,y,width,height,MinValue, MaxValue, Title, Data Array, AutoScale, ChartMode)
    
    DrawGraph(gx + 0 * gap, gy, gwidth, gheight, 900, 1050, Units == "M" ? TXT_PRESSURE_HPA : TXT_PRESSURE_IN, pressure_readings, max_readings, autoscale_on, barchart_off);
    DrawGraph(gx + 1 * gap, gy, gwidth, gheight, 10, 30, Units == "M" ? TXT_TEMPERATURE_C : TXT_TEMPERATURE_F, temperature_readings, max_readings, autoscale_on, barchart_off);
    DrawGraph(gx + 2 * gap, gy, gwidth, gheight, 0, 100, TXT_HUMIDITY_PERCENT, humidity_readings, max_readings, autoscale_off, barchart_off);
    if (SumOfPrecip(rain_readings, max_readings) >= SumOfPrecip(snow_readings, max_readings))
        DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Units == "M" ? TXT_RAINFALL_MM : TXT_RAINFALL_IN, rain_readings, max_readings, autoscale_on, barchart_on);
    else
        DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Units == "M" ? TXT_SNOWFALL_MM : TXT_SNOWFALL_IN, snow_readings, max_readings, autoscale_on, barchart_on);
        */
}

void DisplayConditionsSection(int x, int y, String IconName, IconSize size)
{
    Serial.println("Icon name: " + IconName);
    if (IconName == "01d" || IconName == "01n")
        Sunny(x, y, size, IconName);
    else if (IconName == "02d" || IconName == "02n")
        MostlySunny(x, y, size, IconName);
    else if (IconName == "03d" || IconName == "03n")
        Cloudy(x, y, size, IconName);
    else if (IconName == "04d" || IconName == "04n")
        MostlySunny(x, y, size, IconName);
    else if (IconName == "09d" || IconName == "09n")
        ChanceRain(x, y, size, IconName);
    else if (IconName == "10d" || IconName == "10n")
        Rain(x, y, size, IconName);
    else if (IconName == "11d" || IconName == "11n")
        Tstorms(x, y, size, IconName);
    else if (IconName == "13d" || IconName == "13n")
        Snow(x, y, size, IconName);
    else if (IconName == "50d")
        Haze(x, y, size, IconName);
    else if (IconName == "50n")
        Fog(x, y, size, IconName);
    else
        Nodata(x, y, size, IconName);
}

void DisplayStatusSection(int x, int y, int rssi)
{
    setFont(OpenSans10B);
    DrawRSSI(x + 310, y + 15, rssi);
    DrawBattery(x + 150, y);
}

void DrawPressureAndTrend(int x, int y, float pressure, String slope)
{
    drawString(x + 20, y, String(pressure, (Units == "M" ? 0 : 1)) + (Units == "M" ? "hPa" : "in"), LEFT);
    if (slope == "+")
    {
        DrawSegment(x, y + 10, 0, 0, 8, -8, 8, -8, 16, 0);
        DrawSegment(x - 1, y + 10, 0, 0, 8, -8, 8, -8, 16, 0);
    }
    else if (slope == "0")
    {
        DrawSegment(x, y + 10, 8, -8, 16, 0, 8, 8, 16, 0);
        DrawSegment(x - 1, y + 10, 8, -8, 16, 0, 8, 8, 16, 0);
    }
    else if (slope == "-")
    {
        DrawSegment(x, y + 10, 0, 0, 8, 8, 8, 8, 16, 0);
        DrawSegment(x - 1, y + 10, 0, 0, 8, 8, 8, 8, 16, 0);
    }
}

boolean UpdateLocalTime()
{
    struct tm timeinfo;
    char time_output[30], day_output[30], update_time[30];
    while (!getLocalTime(&timeinfo, 5000))
    { // Wait for 5-sec for time to synchronise
        Serial.println("Failed to obtain time");
        return false;
    }
    CurrentHour = timeinfo.tm_hour;
    CurrentMin = timeinfo.tm_min;
    CurrentSec = timeinfo.tm_sec;
    //See http://www.cplusplus.com/reference/ctime/strftime/
    Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
    if (Units == "M")
    {
        sprintf(day_output, "%s, %02u %s %04u", weekday_D[timeinfo.tm_wday], timeinfo.tm_mday, month_M[timeinfo.tm_mon], (timeinfo.tm_year) + 1900);
        strftime(update_time, sizeof(update_time), "%H:%M:%S", &timeinfo); // Creates: '@ 14:05:49'   and change from 30 to 8 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        sprintf(time_output, "%s", update_time);
    }
    else
    {
        strftime(day_output, sizeof(day_output), "%a %b-%d-%Y", &timeinfo); // Creates  'Sat May-31-2019'
        strftime(update_time, sizeof(update_time), "%r", &timeinfo);        // Creates: '@ 02:05:49pm'
        sprintf(time_output, "%s", update_time);
    }
    Date_str = day_output;
    Time_str = time_output;
    return true;
}

String ConvertUnixTime(int unix_time)
{
    // Returns either '21:12  ' or ' 09:12pm' depending on Units mode
    time_t tm = unix_time;
    struct tm *now_tm = localtime(&tm);
    char output[40];
    if (Units == "M")
    {
        strftime(output, sizeof(output), "%H:%M %d/%m/%y", now_tm);
    }
    else
    {
        strftime(output, sizeof(output), "%I:%M%P %m/%d/%y", now_tm);
    }
    return output;
}

void epd_update()
{
    epd_draw_grayscale_image(epd_full_screen(), framebuffer); // Update the screen
}

double NormalizedMoonPhase(int d, int m, int y)
{
    int j = JulianDate(d, m, y);
    //Calculate approximate moon phase
    double Phase = (j + 4.867) / 29.53059;
    return (Phase - (int)Phase);
}

String MoonPhase(int d, int m, int y, String hemisphere)
{
    int c, e;
    double jd;
    int b;
    if (m < 3)
    {
        y--;
        m += 12;
    }
    ++m;
    c = 365.25 * y;
    e = 30.6 * m;
    jd = c + e + d - 694039.09; /* jd is total days elapsed */
    jd /= 29.53059;             /* divide by the moon cycle (29.53 days) */
    b = jd;                     /* int(jd) -> b, take integer part of jd */
    jd -= b;                    /* subtract integer part to leave fractional part of original jd */
    b = jd * 8 + 0.5;           /* scale fraction from 0-8 and round by adding 0.5 */
    b = b & 7;                  /* 0 and 8 are the same phase so modulo 8 for 0 */
    if (hemisphere == "south")
        b = 7 - b;
    if (b == 0)
        return TXT_MOON_NEW; // New;              0%  illuminated
    if (b == 1)
        return TXT_MOON_WAXING_CRESCENT; // Waxing crescent; 25%  illuminated
    if (b == 2)
        return TXT_MOON_FIRST_QUARTER; // First quarter;   50%  illuminated
    if (b == 3)
        return TXT_MOON_WAXING_GIBBOUS; // Waxing gibbous;  75%  illuminated
    if (b == 4)
        return TXT_MOON_FULL; // Full;            100% illuminated
    if (b == 5)
        return TXT_MOON_WANING_GIBBOUS; // Waning gibbous;  75%  illuminated
    if (b == 6)
        return TXT_MOON_THIRD_QUARTER; // Third quarter;   50%  illuminated
    if (b == 7)
        return TXT_MOON_WANING_CRESCENT; // Waning crescent; 25%  illuminated
    return "";
}

void Convert_Readings_to_Imperial()
{ // Only the first 3-hours are used
    WxConditions[0].Pressure = hPa_to_inHg(WxConditions[0].Pressure);
    WxForecast[0].Rainfall = mm_to_inches(WxForecast[0].Rainfall);
    WxForecast[0].Snowfall = mm_to_inches(WxForecast[0].Snowfall);
}

float mm_to_inches(float value_mm)
{
    return 0.0393701 * value_mm;
}

float hPa_to_inHg(float value_hPa)
{
    return 0.02953 * value_hPa;
}

int JulianDate(int d, int m, int y)
{
    int mm, yy, k1, k2, k3, j;
    yy = y - (int)((12 - m) / 10);
    mm = m + 9;
    if (mm >= 12)
        mm = mm - 12;
    k1 = (int)(365.25 * (yy + 4712));
    k2 = (int)(30.6001 * mm + 0.5);
    k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;
    // 'j' for dates in Julian calendar:
    j = k1 + k2 + d + 59 + 1;
    if (j > 2299160)
        j = j - k3; // 'j' is the Julian date at 12h UT (Universal Time) For Gregorian calendar:
    return j;
}


// Tasks //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void WebServerTask(void *pvParameters)
{
    Serial.println("Starting web server for configuration...");
    setupWEB(); // Start the web server
    Serial.println("Web server set up. Waiting for configuration...");
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for the server to start
    xSemaphoreGive(configSemaphore); // Signal the config task to continue
    vTaskDelete(NULL); // Delete the task when done
}

void ConfigTask(void *pvParameters) 
{
    // OpenWeather configuration setup / reading
    if (digitalRead(!USR_BUTTON) || !SPIFFS.exists("/config.json"))
    {
        Serial.println("BUTTON Pressed or config file missing. Starting web server...");
        xTaskCreate(WebServerTask, "WebServerTask", 8192, NULL, 1, NULL);
        xSemaphoreTake(configSemaphore, portMAX_DELAY); // Wait for the web server to finish
    }
    
    else
    {
        StaticJsonDocument<1024> json1;
        File configfile = SPIFFS.open("/config.json", "r");
        DeserializationError error = deserializeJson(json1, configfile);

        if (error)
        {
            Serial.println("Failed to read file, using default configuration");
        }
        else
        {
        // Extract configuration values
        ssid = json1["WLAN"]["ssid"].as<String>();
        password = json1["WLAN"]["password"].as<String>();

        apikey     = json1["OpenWeather"]["apikey"].as<String>();
        server     = json1["OpenWeather"]["server"].as<String>();
        Country    = json1["OpenWeather"]["country"].as<String>();
        City       = json1["OpenWeather"]["city"].as<String>();
        Hemisphere = json1["OpenWeather"]["hemisphere"].as<String>();
        Units      = json1["OpenWeather"]["units"].as<String>();

        ntpServer = json1["ntp"]["server"].as<String>();
        Timezone = json1["ntp"]["timezone"].as<String>();

        // WakeupHour = strtok(json1["schedule_power"]["on_time"].as<const char *>());
        // SleepHour  = json1["schedule_power"]["off_time"].as<String>();
        Serial.println("Config loaded successfully.");
        }

        configfile.close();
    }
    xSemaphoreGive(configSemaphore); // Signal the main task to continue
    vTaskDelete(NULL); // Delete the task when done - first boot
}

void SHT4xReadTask(void *pvParameters)
{
    static sht4x_t dev;
    memset(&dev, 0, sizeof(sht4x_t));

    ESP_ERROR_CHECK(sht4x_init_desc(&dev, 0, I2C_MASTER_SDA, I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sht4x_init(&dev));

    TaggedSensorData sht4xdata;
    sht4xdata.tag = SENSOR_SHT4X;
    sht4xdata.pressure = 0;

    TickType_t last_wakeup = xTaskGetTickCount();

    // get the measurement duration for high repeatability;
    uint8_t duration = sht4x_get_measurement_duration(&dev);
    while (1)
    {   
        // Wait for the semaphore to be given by the main task to start measurement
        ESP_LOGI("SHT4X", "Waiting for semaphores...");
        xSemaphoreTake(SHT4XTriggerSem, portMAX_DELAY);
        ESP_LOGI("SHT4X", "Waiting for MUTEX...");
        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        // Trigger one measurement in single shot mode with high repeatability.
        ESP_ERROR_CHECK(sht4x_start_measurement(&dev));
        // Wait until measurement is ready (duration returned from *sht4x_get_measurement_duration*).
        vTaskDelay(duration); // duration is in ticks

        // retrieve the values and send it to the queue
        //ESP_ERROR_CHECK(sht4x_get_results(&dev, &sht4xdata.temperature, &sht4xdata.humidity));
        if(sht4x_get_results(&dev, &sht4xdata.temperature, &sht4xdata.humidity) == ESP_OK)
        {
            ESP_LOGI("SHT40", "Timestamp: %lu, SHT40  - Temperature: %.2f °C, Humidity: %.2f %%", (unsigned long)xTaskGetTickCount(), sht4xdata.temperature, sht4xdata.humidity);
            if (xQueueSend(sensorDataQueue, &sht4xdata, pdMS_TO_TICKS(2000)) != pdPASS) 
            {
                ESP_LOGI("SHT40", "Failed to send data to sensorDataQueue in time");
            }
            else
            {
                ESP_LOGI("SHT40", "SHT40 Data sent to sensorDataQueue");
            }
        }
        else
        {
            ESP_LOGI("SHT40", "SHT40 Failed to read sensor data.");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        xSemaphoreGive(i2cMutex);
        xSemaphoreGive(sensorDataReadySem);
    }
}

void BME280ReadTask(void *pvParameters)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);

    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, I2C_MASTER_SDA, I2C_MASTER_SCL));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", (bme280p ? "BME280" : "BMP280"));

    TaggedSensorData bme280data;
    bme280data.tag = SENSOR_BME280;

    while (1)
    {
        ESP_LOGI("BME280", "Waiting for semaphores...");
        xSemaphoreTake(BME280TriggerSem, portMAX_DELAY);
        ESP_LOGI("BME280", "Waiting for MUTEX");
        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        // Set the sensor to forced mode to initiate a measurement
        ESP_ERROR_CHECK(bmp280_force_measurement(&dev));

        // Wait for the measurement to complete
        bool busy;
        do
        {
            ESP_ERROR_CHECK(bmp280_is_measuring(&dev, &busy));
            if (busy)
            vTaskDelay(pdMS_TO_TICKS(5)); // Wait for 5ms before checking again
        } while (busy);

        // Read the measurement results
        if (bmp280_read_float(&dev, &bme280data.temperature, &bme280data.pressure, &bme280data.humidity) == ESP_OK)
        {
            //printf("Timestamp: %lu, BME280 - Temperature: %.2f °C, Humidity: %.2f %%, Pressure: %.2f hPa\n",(unsigned long)xTaskGetTickCount(), bme280data.temperature, bme280data.humidity, bme280data.pressure/100); // Pressure in hPa
            ESP_LOGI("BME280", "Timestamp: %lu, BME280 - Temperature: %.2f °C, Humidity: %.2f %%, Pressure: %.2f hPa", (unsigned long)xTaskGetTickCount(), bme280data.temperature, bme280data.humidity, bme280data.pressure/100);
            //printf("BME280 task high watermark: %u\n", uxTaskGetStackHighWaterMark(NULL)); // checking stack size
            // Send data to the queue
            if (xQueueSend(sensorDataQueue, &bme280data, pdMS_TO_TICKS(2000)) != pdPASS) 
            {
                ESP_LOGI("BME280", "Failed to send data to sensorDataQueue in time");
            }
            else
            {
                ESP_LOGI("BME280", "BME280 Data sent to sensorDataQueue");
            }
        }
        else
        {
            ESP_LOGI("BME280", "BME280 Failed to read sensor data.");
        }
        // Since the sensor automatically goes to sleep after forced mode, we don't need an explicit sleep mode call.
        xSemaphoreGive(i2cMutex);
        xSemaphoreGive(sensorDataReadySem);
    }
}

void ProcessSensorDataTask(void *pvParameters) 
{
    TaggedSensorData bme280data = {SENSOR_NONE, 0, 0, 0};
    TaggedSensorData sht4xdata = {SENSOR_NONE, 0, 0, 0};
    TaggedSensorData receivedData = {SENSOR_NONE, 0, 0, 0};

    UntaggedSensorData lastAverage = {0}; 
    UntaggedSensorData historicalAverage = {0, 0, 0};
    UntaggedSensorData calculationData = {0};

    bool bme280Ready = false; 
    bool sht4xReady = false;

    TickType_t startTime;

    while (1) 
    {
        // Wait for both sensors to be ready (3 sec)
        ESP_LOGI("DATA", "Waiting for semaphores...");
        xSemaphoreTake(sensorDataReadySem, portMAX_DELAY); 
        xSemaphoreTake(sensorDataReadySem, pdMS_TO_TICKS(1000));

        // Wait 3 seconds TOTAL for both sensors
        startTime = xTaskGetTickCount();
        while ((!bme280Ready || !sht4xReady) && ((xTaskGetTickCount() - startTime) < pdMS_TO_TICKS(2000))) // Unless both sensors data received waits for 3 seconds total (1000ms in semaphore delay already)
        {
            if (xQueueReceive(sensorDataQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE) // check if both sensors data were received every 100ms
            {
                if (receivedData.tag == SENSOR_BME280) 
                {  // BME280
                    bme280data = receivedData;
                    bme280Ready = true;
                    ESP_LOGI("DATA", "BME280 data received");
                }
                else if (receivedData.tag == SENSOR_SHT4X) 
                {  // SHT4X
                    sht4xdata = receivedData;
                    sht4xReady = true;
                    ESP_LOGI("DATA", "SHT4X data received");
                }
            }
        }
        // Process data when BOTH sensors are ready
        if (bme280Ready && sht4xReady) 
        {
            calculationData.temperature = (bme280data.temperature + sht4xdata.temperature) / 2.0; // Average
            calculationData.humidity = ((bme280data.humidity * 2) + sht4xdata.humidity) / 3.0;   // Weighted average in favour of BME280
            calculationData.pressure = bme280data.pressure / 100;                               // Pressure from BME280 only
            bme280Ready = false; // Reset readiness flags
            sht4xReady = false;
            ESP_LOGI("DATA", "Both sensor data available.");
        }
        // Process data when only BME280 is ready
        else if (bme280Ready && !sht4xReady) 
        {
            calculationData.temperature = bme280data.temperature;
            calculationData.humidity = bme280data.humidity;
            calculationData.pressure = bme280data.pressure / 100;
            bme280Ready = false; // Reset readiness flag
            ESP_LOGW("DATA", "Only BME280 data available.");
        }
        // Process data when only SHT40 is ready
        else if (!bme280Ready && sht4xReady) 
        {
            calculationData.temperature = sht4xdata.temperature;
            calculationData.humidity = sht4xdata.humidity;
            calculationData.pressure = WxConditions[0].Pressure; // If only SHT40 available, using OpenWeather pressure
            sht4xReady = false; // Reset readiness flag
            ESP_LOGW("DATA", "Only SHT40 data available.");
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            ESP_LOGE("DATA", "No sensor data available.");
            continue;
        }

        // Update historical data
        xSemaphoreTake(historyCalcMutex, portMAX_DELAY);
        readingHistory[historyIndex] = calculationData; // Update history with latest data
        historyIndex = (historyIndex + 1) % 3; //circular loop 0 -> 1 -> 2 -> 0, update up to 3 slots of historyIndex 
        numReadings = numReadings < 3 ? numReadings + 1 : 3; // Update number of readings, for checking if bias applies (>=3)

        // Calculate historical average
        if (numReadings > 1) 
        {
            for (int i = 0; i < numReadings; i++) 
            {
                historicalAverage.temperature += readingHistory[i].temperature;
                historicalAverage.humidity += readingHistory[i].humidity;
                historicalAverage.pressure += readingHistory[i].pressure;
            }
            historicalAverage.temperature /= numReadings;
            historicalAverage.humidity /= numReadings;
            historicalAverage.pressure /= numReadings;
        }

        // Apply bias with weighted average
        if (numReadings >= 3) 
        {
            processedResult.temperature = ((4 * calculationData.temperature) + historicalAverage.temperature)/5;
            processedResult.humidity = ((4 * calculationData.humidity) + historicalAverage.humidity)/5;
            processedResult.pressure = (((4 * calculationData.pressure) + historicalAverage.pressure)/5); // Pressure in hPa
        } 
        else 
        {
            processedResult = calculationData;
        }

        xSemaphoreGive(historyCalcMutex);
        ESP_LOGI("DATA", "Processed Data: Temp=%.2f, Hum=%.2f, Press=%.2f", processedResult.temperature, processedResult.humidity, processedResult.pressure);

        // Pass processed data to the drawing function
        if (xQueueSend(processedDataQueue, &processedResult, portMAX_DELAY) != pdTRUE) 
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            ESP_LOGW("DATA", "Failed to enqueue processed data.");
        } 
        else
        {
            ESP_LOGI("DATA", "Processed data enqueued.");
        }
        xSemaphoreGive(dataProcessedSem);
    }
}

void WeatherUpdateTask(void *pvParameters)
{
    while (1)
    {
        if (StartWiFi() == WL_CONNECTED && SetTime() == true)
        {
            WiFiClient client;
            bool RxWeather = false;
            bool RxForecast = false;

            for (int attempts = 0; attempts < 2 && (!RxWeather || !RxForecast); ++attempts)
            {
                if (!RxWeather)
                    RxWeather = obtainWeatherData(client, "weather");
                if (!RxForecast)
                    RxForecast = obtainWeatherData(client, "forecast");
            }
                    // Trigger sensor readings
        xSemaphoreGive(SHT4XTriggerSem);
        xSemaphoreGive(BME280TriggerSem);
            
            //if (RxWeather && RxForecast)
            //{   
                    Serial.println("Updating display...");
                    epd_poweron();
                    Serial.println("EPD POWER ON");
                    epd_clear();
                    Serial.println("EPD CLEAR");

                    if(xSemaphoreTake(dataProcessedSem, pdMS_TO_TICKS(10000)) != pdTRUE)
                    {
                        Serial.println("Failed to take dataProcessedMutex");
                    }
                    DisplayWeather(screenState);
                    epd_update();
                    epd_poweroff_all();
                    // Serial.println("Stack high watermark: " + String(uxTaskGetStackHighWaterMark(NULL)));
                    // Serial.println("Free heap: " + String(esp_get_free_heap_size()));
            //}
        }
        Serial.println("Initiating Sleep...");
        memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
        InitiateSleep(); // Light sleep by default rn
        vTaskDelay(pdMS_TO_TICKS(500));
        //vTaskDelay(MINUTES_TO_TICKS(SleepDuration)); //- TESTS ONLY with InitiateSleep OFF
        

    }
}

void setup()
{
    InitialiseSystem();
    SPIFFS.begin();

    // Configure button interrupt
    pinMode(USR_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(USR_BUTTON), handleButtonPress, RISING);


    // Create queues with error checking
    sensorDataQueue = xQueueCreate(10, sizeof(TaggedSensorData));
    processedDataQueue = xQueueCreate(10, sizeof(UntaggedSensorData));
    if (sensorDataQueue == NULL || processedDataQueue == NULL) 
    {
        ESP_LOGE("SETUP", "Failed to create queues");
        return;
    }
    ESP_LOGI("SETUP", "All queues created successfully");


    // Create semaphores with error checking
    configSemaphore = xSemaphoreCreateBinary();
    SHT4XTriggerSem = xSemaphoreCreateBinary();
    BME280TriggerSem = xSemaphoreCreateBinary();
    sensorDataReadySem = xSemaphoreCreateCounting(2, 0);
    dataProcessedSem = xSemaphoreCreateBinary();

    historyCalcMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex();

    if (!configSemaphore || !BME280TriggerSem || !SHT4XTriggerSem || !sensorDataReadySem || !dataProcessedSem || !historyCalcMutex || !i2cMutex) 
    {
        ESP_LOGE("SETUP", "Failed to create semaphores");
        return;
    }
    ESP_LOGI("SETUP", "All semaphores created successfully");
    



    // Create tasks with error checking
    BaseType_t xReturned;
    xReturned = xTaskCreate(ConfigTask, "ConfigTask", 8192, NULL, 1, NULL);
    if (xReturned != pdPASS) 
    {
        ESP_LOGE("SETUP", "Failed to create Config Task");
        return;
    }
    
    xSemaphoreTake(configSemaphore, portMAX_DELAY); // Wait for the config task to finish

    xReturned = xTaskCreate(BME280ReadTask, "BME280ReadTask", 4096, NULL, 2, NULL);
    if (xReturned != pdPASS) 
    {
        ESP_LOGE("SETUP", "Failed to create BME280 task");
        return;
    }

    xReturned = xTaskCreate(SHT4xReadTask, "SHT4xReadTask", 4096, NULL, 2, NULL);
    if (xReturned != pdPASS) 
    {
        ESP_LOGE("SETUP", "Failed to create SHT4x task");
        return;
    }

    xReturned = xTaskCreate(ProcessSensorDataTask, "ProcessData", 8192, NULL, 3, NULL);
    if (xReturned != pdPASS) 
    {
        ESP_LOGE("SETUP", "Failed to create ProcessData task");
        return;
    }

    xReturned = xTaskCreate(WeatherUpdateTask, "WeatherUpdateTask", 8192, NULL, 4, NULL);
    if (xReturned != pdPASS) 
    {
        ESP_LOGE("SETUP", "Failed to create WeatherUpdate Task");
        return;
    }
    ESP_LOGI("SETUP", "All tasks created successfully");
    delay(100);
}

void loop()
{
    if (ButtonPressed) {
        Serial.println("Button pressed");
        ButtonPressed = false; // Reset the flag
        screenState = (screenState + 1) % 3; // Cycle through the 3 screens
        Serial.println("Screen state: " + String(screenState));
    }
}
