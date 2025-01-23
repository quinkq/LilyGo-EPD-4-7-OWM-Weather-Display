
#ifndef LILYGO_EPD_47_OWM_WEATHER_DISPLAY_H
#define LILYGO_EPD_47_OWM_WEATHER_DISPLAY_H

#include <Arduino.h>           // In-built
#include "epd_driver.h"        // https://github.com/Xinyuan-LilyGO/LilyGo-EPD47
#include "drawingFunctions.h"

void InitialiseDisplay();
void InitialiseSystem();
void InitiateSleep();
boolean SetTime();
uint8_t StartWiFi();
void StopWiFi();

void DisplayGeneralInfoSection();
void DisplayWeatherIcon(int x, int y);
void DisplayMainWeatherSection(int x, int y);
void DisplayDisplayWindSection(int x, int y, float angle, float windspeed, int Cradius);

String WindDegToOrdinalDirection(float winddirection);

void DisplayTemperatureSection(int x, int y);
void DisplaySensorReadings(int x, int y);
void DisplaySensorReadingsGarden(int x, int y);
void DisplaySensorReadingsRoom(int x, int y);
void DisplayForecastTextSection(int x, int y);
void DisplayPressureSection(int x, int y, float pressure, String slope);
void DisplayForecastWeather(int x, int y, int index);
void DisplayAstronomySection(int x, int y);

String MoonPhase(int d, int m, int y, String hemisphere);

void DisplayForecastSection(int x, int y);
void DisplayConditionsSection(int x, int y, String IconName, IconSize size);
void DrawPressureAndTrend(int x, int y, float pressure, String slope);

void DisplayStatusSection(int x, int y, int rssi);

boolean UpdateLocalTime();
String ConvertUnixTime(int unix_time);

void edp_update();

void Convert_Readings_to_Imperial();
float mm_to_inches(float value_mm);
float hPa_to_inHg(float value_hPa);
double NormalizedMoonPhase(int d, int m, int y);
int JulianDate(int d, int m, int y);

#endif // #endif