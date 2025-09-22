#ifndef _RTC_HELPER_
#define _RTC_HELPER_

// defined in SensorLib
#include "SensorPCF85063.hpp"// RTC

extern void RTC_Setup();
extern void RTC_Loop();
extern SensorPCF85063 GetRTC(void);
extern void RTC_SetDateTime(int year, int month, int day, int hour, int minute, int second);
extern RTC_DateTime RTC_GetDateTime();

#endif
