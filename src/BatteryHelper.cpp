/*
 * BatteryHelper.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#include "SoCHelper.h"
#include "BatteryHelper.h"
#include "SkyView.h"
#include "TFTHelper.h"
#include "WebHelper.h"
#include "Arduino_DriveBus_Library.h"
#include "Platform_ESP32.h"

static unsigned long Battery_TimeMarker = 0;

static float Battery_voltage_cache      = 0;
static int Battery_cutoff_count         = 0;
static int charging_status_cache        = 0;
float voltage_start = 0;
unsigned long start_time_ms = 0;

float voltage_end = 0;
unsigned long end_time_ms = 0;

std::unique_ptr<Arduino_SY6970> SY6970;

void SY6970_setup() {

  SY6970 = std::unique_ptr<Arduino_SY6970>(new Arduino_SY6970(IIC_Bus, SY6970_DEVICE_ADDRESS, DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));
  if (!SY6970->begin()) {
    Serial.println("SY6970 init failed");
    while (1);
  }
  Serial.println("SY6970 init success");
  Serial.println("SY6970 setup Chrging parameters");
  SY6970->IIC_Write_Device_State(SY6970->Arduino_IIC_Power::Device::POWER_DEVICE_ADC_MEASURE, SY6970->Arduino_IIC_Power::Device_State::POWER_DEVICE_ON);
  SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_WATCHDOG_TIMER, 0);
  Serial.println("SY6970 Device ON and set Watchdog Timer to 0");
   // Set fast charging current limit
  SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_FAST_CHARGING_CURRENT_LIMIT, 200);
  // Set precharge charging current limit
  SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_PRECHARGE_CHARGING_CURRENT_LIMIT, 50);
  // Set termination charging current limit
  SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_TERMINATION_CHARGING_CURRENT_LIMIT, 50);
}                                                       

int read_SY6970_voltage()
{
  int voltage = SY6970->IIC_Read_Device_Value(SY6970->Arduino_IIC_Power::Value_Information::POWER_BATTERY_VOLTAGE);
  if (voltage < 0) {
    return 0;
  }
  return voltage;
}

int read_SY6970_charge_current() {
  int current = SY6970->IIC_Read_Device_Value(SY6970->Arduino_IIC_Power::Value_Information::POWER_CHARGING_CURRENT);
  if (current < 0) {
    return 0;
  }
  
  return current;
}

int read_SY6970_charging_status()
{
  String status = SY6970->IIC_Read_Device_State(SY6970->Arduino_IIC_Power::Status_Information::POWER_CHARGING_STATUS);

  if (status == "Not Charging") return 0;
  else if (status == "Pre-Charge") return 1;
  else if (status == "Fast Charging") return 2;
  else if (status == "Charge Termination Done") return 3;
  else return -1; // Unknown
}

void Battery_setup()
{
  SoC->Battery_setup();

  Battery_voltage_cache = voltage_start = SoC->Battery_voltage();
  start_time_ms = millis();
}

float Battery_voltage()
{
  return Battery_voltage_cache;
}
int charging_status() {
  return charging_status_cache;
}
/* low battery voltage threshold */
float Battery_threshold()
{
  return BATTERY_THRESHOLD_LIPO;
}

/* Battery is empty */
float Battery_cutoff()
{
  return BATTERY_CUTOFF_LIPO;
}
void battery_fini() {
  voltage_end = SoC->Battery_voltage();
  end_time_ms = millis();

  unsigned long durationSeconds = (end_time_ms - start_time_ms) / 1000;

  float voltage_drop = voltage_start - voltage_end;
  // Simplified estimate: assume linear discharge, estimate mAh used
  float estimated_mAh = (voltage_drop / 0.001) * 0.0015f; // tuning factor as needed
  writeBatteryLog(voltage_start, voltage_end, durationSeconds, estimated_mAh);

}

void Battery_loop()
{
  if (isTimeToBattery()) {
    if (!SY6970) {
      Serial.println("Error: SY6970 not initialized!");
      return;
  }
    float voltage = SoC->Battery_voltage();
    int charging_status = read_SY6970_charging_status();

    if ( hw_info.model    == SOFTRF_MODEL_SKYVIEW &&
        (hw_info.revision == HW_REV_H741_01)) {

      if (voltage > 2.0 && voltage < Battery_cutoff()) {
        if (Battery_cutoff_count > 3) {
          ESP32_TFT_fini("LOW BATTERY");
          shutdown("LOW BATTERY");
        } else {
          Battery_cutoff_count++;
        }
      } else {
        Battery_cutoff_count = 0;
      }
    }

    Battery_voltage_cache = voltage;
    charging_status_cache = charging_status;
    Battery_TimeMarker = millis();
  }
}
