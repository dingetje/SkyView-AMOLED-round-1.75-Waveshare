#ifdef WAVESHARE_AMOLED_1_75

#include "RTCHelper.h"
#include "../pins_config.h"
#include <DebugLog.h>

// defined in SensorLib
static SensorPCF85063 rtc;
static uint32_t lastMillis;

void RTC_SetDateTime(int year, int month, int day, int hour, int minute, int second)
{
    rtc.setDateTime(year, month, day, hour, minute, second);
}

void RTC_Setup()
{
  Serial.println("Initializing RTC...");
  if (!rtc.begin(Wire,PCF85063_SLAVE_ADDRESS,IIC_SDA,IIC_SCL))
  {
    Serial.println("Failed to find PCF8563 RTC - check your wiring!");
    return;
  }
  Serial.println("RTC started!");
}

SensorPCF85063 GetRTC()
{
  return rtc;
}

RTC_DateTime RTC_GetDateTime()
{
  return rtc.getDateTime();
}

void RTC_Loop()
{
    if (millis() - lastMillis > 10000) {
      char buf[30];
      lastMillis = millis();
      RTC_DateTime now = rtc.getDateTime();

      snprintf(buf, sizeof(buf), "%4d/%02d/%02d %02d:%02d:%02d", now.year, now.month, now.day, now.hour, now.minute, now.second);
      PRINTLN("Current time: " + String(buf));
    }
}
#endif