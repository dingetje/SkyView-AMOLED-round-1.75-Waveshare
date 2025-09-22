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
#include <DebugLog.h>

// for Waveshare AMOLED 1.75 inch round display
#if defined(XPOWERS_CHIP_AXP2101)
#include "XPowersLib.h"
#include "ESP_IOExpander_Library.h"

#define _EXAMPLE_CHIP_CLASS(name, ...) ESP_IOExpander_##name(__VA_ARGS__)
#define EXAMPLE_CHIP_CLASS(name, ...) _EXAMPLE_CHIP_CLASS(name, ##__VA_ARGS__)

uint32_t printTime = 0;

XPowersPMU power;
ESP_IOExpander *expander = NULL;

bool pmu_irq = false;
void setPmuIrqFlag(void)
{
    pmu_irq = true;
}

#endif

static unsigned long Battery_TimeMarker = 0;

static float Battery_voltage_cache      = 0;
static int Battery_cutoff_count         = 0;
static int charging_status_cache        = 0;
float voltage_start = 0;
unsigned long start_time_ms = 0;

float voltage_end = 0;
unsigned long end_time_ms = 0;

#if defined(SY6970)
std::unique_ptr<Arduino_SY6970> SY6970;

void SY6970_setup() {

  SY6970 = std::unique_ptr<Arduino_SY6970>(new Arduino_SY6970(IIC_Bus, SY6970_DEVICE_ADDRESS, DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));
  if (!SY6970->begin()) {
    PRINTLN("SY6970 init failed");
    while (1);
  }
  PRINTLN("SY6970 init success");
  PRINTLN("SY6970 setup Chrging parameters");
  SY6970->IIC_Write_Device_State(SY6970->Arduino_IIC_Power::Device::POWER_DEVICE_ADC_MEASURE, SY6970->Arduino_IIC_Power::Device_State::POWER_DEVICE_ON);
  SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_WATCHDOG_TIMER, 0);
  PRINTLN("SY6970 Device ON and set Watchdog Timer to 0");
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
#endif


#if defined(XPOWERS_CHIP_AXP2101)
static void adcOn() {
  power.enableTemperatureMeasure();
  // Enable internal ADC detection
  power.enableBattDetection();
  power.enableVbusVoltageMeasure();
  power.enableBattVoltageMeasure();
  power.enableSystemVoltageMeasure();
}

/*
static void adcOff() {
  power.disableTemperatureMeasure();
  // Enable internal ADC detection
  power.disableBattDetection();
  power.disableVbusVoltageMeasure();
  power.disableBattVoltageMeasure();
  power.disableSystemVoltageMeasure();
}
*/

void prepare_AXP2101_deep_sleep()
{
  LOG_DEBUG("Prepare AXP2101 PMU for deep sleep wake up...");
  power.clearIrqStatus();
  // Enable the required interrupt function
  power.enableIRQ(
     XPOWERS_AXP2101_PKEY_SHORT_IRQ  | XPOWERS_AXP2101_PKEY_LONG_IRQ  //POWER KEY
  );
  // Enable PMU sleep
  power.enableSleep();

  // Reserve the MCU chip power supply, AXP2101 usually uses DC as ESP power supply
  power.enableDC1();

  // Turn off the power output of other channels
  power.disableDC2();
  power.disableDC3();
  power.disableDC4();
  power.disableDC5();
  power.disableALDO1();
  power.disableALDO2();
  power.disableALDO3();
  power.disableALDO4();
  power.disableBLDO1();
  power.disableBLDO2();
  power.disableCPUSLDO();
  power.disableDLDO1();
  power.disableDLDO2();

  // Clear PMU Interrupt Status Register
  power.clearIrqStatus();

  // Send IRQ wakeup command
//  power.enableWakeup();
}

#if defined(I2C_SCAN)
static void I2C_Scan()
{
  byte error, address;
  int nDevices;

  PRINTLN("==============================================================");  
  PRINTLN("Scanning I2C bus...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      PRINT("I2C device found at address 0x");
      if (address<16)
        PRINT("0");
      PRINT(address,HEX);
      PRINTLN("  !");

      nDevices++;
    }
    else if (error==4)
    {
      PRINT("Unknown error at address 0x");
      if (address<16)
        PRINT("0");
      PRINTLN(address,HEX);
    }
  }
  if (nDevices == 0)
    PRINTLN("No I2C devices found!");
  else
    PRINTLN("done!");  

  PRINTLN("==============================================================");  
}
#endif

void AXP2101_ChkIRQ()
{
  int chk_pmu_irq = expander->digitalRead(AXP_IRQ); // HIGH or LOW?
  if (chk_pmu_irq == HIGH) 
  {
    // handled in loop
    setPmuIrqFlag();
  }
}

void AXP2101_setup()
{
  // already been here?
  if (expander != NULL)
  {
    return;
  }

  Wire.begin(IIC_SDA, IIC_SCL);

#if defined(I2C_SCAN)
  I2C_Scan();
#endif

  LOG_DEBUG("Create ESP_IOExpander object...");
  expander = new EXAMPLE_CHIP_CLASS(TCA95xx_8bit,
                                    (i2c_port_t)0, 
                                    ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                                    IIC_SCL, IIC_SDA);

  expander->begin();
  expander->pinMode(AXP_IRQ, INPUT);
  expander->pinMode(PWR_BUTTON, INPUT);
  expander->printStatus();

  bool result = power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  if (result == false) 
  {
    PRINTLN("AXP2101 PMU is not online...");
    return;
  }

  power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);

  // Clear all interrupt flags
  power.clearIrqStatus();
  // Enable the required interrupt function
  power.enableIRQ(
      XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
      XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
      XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       |   //POWER KEY
      XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
  );

  // Set the minimum common working voltage of the PMU VBUS input,
  // below this value will turn off the power
  power.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
  // Set the maximum current of the PMU VBUS input,
  // higher than this value will turn off the PMU
  power.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);
  // Set VSY off voltage as 2600mV , Adjustment range 2600mV ~ 3300mV
  power.setSysPowerDownVoltage(2600);
  // Set the precharge charging current
  power.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
  // Set constant current charge current limit
  power.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
  // Set stop charging termination current
  power.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
  // Set charge cut-off voltage
  power.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

  LOG_DEBUG("Read ESP_IOExpander IRQ pin of AXP2101...");
  AXP2101_ChkIRQ();

  // power OLED
  power.setBLDO1Voltage(3300);
  power.enableALDO1();
  LOG_DEBUG("AXP2101 Power chip initialized");

  adcOn();
}

void printPMU()
{
    PRINTLN("---------------------------------------------------------------------------------------------------------");
    PRINTLN("Satus1  Satus2  CHARG   DISC   STBY    VBUSIN    VGOOD    VBAT   VBUS   VSYS   Percentage    CHG_STATUS");
    PRINTLN("(Bin)   (Bin)   (bool)  (bool) (bool)  (bool)    (bool)   (mV)   (mV)   (mV)      (%)           (str)  ");
    PRINTLN("---------------------------------------------------------------------------------------------------------");
    uint16_t statusVal =  power.status();
    PRINT("0b"); PRINT(statusVal >> 8, BIN); PRINT("\t");
    PRINT("0b"); PRINT(statusVal & 0xFF, BIN); PRINT("\t");
    PRINT(power.isCharging() ? "YES" : "NO "); PRINT("\t");
    PRINT(power.isDischarge() ? "YES" : "NO "); PRINT("\t");
    PRINT(power.isStandby() ? "YES" : "NO "); PRINT("\t");
    PRINT(power.isVbusIn() ? "YES" : "NO "); PRINT("\t");
    PRINT(power.isVbusGood() ? "YES" : "NO "); PRINT("\t");
    PRINT(power.getBattVoltage());     PRINT("\t");
    PRINT(power.getVbusVoltage());     PRINT("\t");
    PRINT(power.getSystemVoltage());   PRINT("\t");

    // The battery percentage may be inaccurate at first use, the PMU will automatically
    // learn the battery curve and will automatically calibrate the battery percentage
    // after a charge and discharge cycle
    PRINT(power.getBatteryPercent()); PRINT("\t");

    uint8_t charge_status = power.getChargerStatus();
    if (charge_status == XPOWERS_AXP2101_CHG_TRI_STATE) {
        PRINTLN("tri_charge");
    } else if (charge_status == XPOWERS_AXP2101_CHG_PRE_STATE) {
        PRINTLN("pre_charge");
    } else if (charge_status == XPOWERS_AXP2101_CHG_CC_STATE) {
        PRINTLN("constant charge(CC)");
    } else if (charge_status == XPOWERS_AXP2101_CHG_CV_STATE) {
        PRINTLN("constant voltage(CV)");
    } else if (charge_status == XPOWERS_AXP2101_CHG_DONE_STATE) {
        PRINTLN("charge done");
    } else if (charge_status == XPOWERS_AXP2101_CHG_STOP_STATE) {
        PRINTLN("not charge");
    }
    PRINTLN();
}

float read_AXP2101_voltage()
{
  return power.getBattVoltage(); // in mV
}

uint8_t read_AXP2101_charging_status()
{
  uint8_t charge_status = power.getChargerStatus();

  switch (charge_status) {
    case XPOWERS_AXP2101_CHG_TRI_STATE: // ??
      return 0; // Not charging
    case XPOWERS_AXP2101_CHG_PRE_STATE:
      return 1; // Pre-Charge
    case XPOWERS_AXP2101_CHG_CV_STATE:
    case XPOWERS_AXP2101_CHG_CC_STATE:
      return 2; // Fast Charging
    case XPOWERS_AXP2101_CHG_DONE_STATE:
      return 3; // Charge Termination Done
      break;
    case XPOWERS_AXP2101_CHG_STOP_STATE:
      return 0; // Not charging
  }
  return -1; // Unknown
}

int read_AXP2101_charge_current() 
{
  return power.getChargerConstantCurr();
}
#endif

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
  // TBD: determine fudge factor for Waveshare board
  float estimated_mAh = (voltage_drop / 0.001) * 0.0015f; // tuning factor as needed
  writeBatteryLog(voltage_start, voltage_end, durationSeconds, estimated_mAh);

}

void Battery_loop()
{
#if defined(XPOWERS_CHIP_AXP2101)
  static int pwr_button_last = 0;

  if (millis() > printTime) 
  {
      // every ~30 sec.
      printTime = millis() + 30000;
      printPMU();
  }

  // PWR button is tied to IO Expander pin 4
  int pwr_button = expander->digitalRead(PWR_BUTTON);
  if (pwr_button != pwr_button_last) {
      LOG_DEBUG("PWR button state changed: " + String(pwr_button));
      // TODO: button press not tied to anything yet
      pwr_button_last = pwr_button;
  }

  // Waveshare has tied the PMU IRQ to an IO Expander pin too
  // which means we need to poll the IO Expander to find out if
  // Power chip has raised an interrupt...
  AXP2101_ChkIRQ();

  if (pmu_irq) 
  {
    pmu_irq = false;
    char buf[8];
    // Get PMU Interrupt Status Register
    uint32_t status = power.getIrqStatus();
    snprintf(buf,sizeof(buf), "0x%04X",status);
    LOG_DEBUG("AXP2101 IRQ status reg = " + String(buf));
    if (power.isPekeyLongPressIrq()) 
    {
      LOG_DEBUG("PWR button long press!");
    }
    power.clearIrqStatus();
  }
#endif

   // Read battery voltage and charging status every 30 seconds
  if (isTimeToBattery()) 
  {

  #if defined(SY6970)
    if (!SY6970) {
      PRINTLN("Error: SY6970 not initialized!");
      return;
  }
  #endif

  #if defined(SY6970)
    int charging_status = read_SY6970_charging_status();
  #elif defined(XPOWERS_CHIP_AXP2101)
    int charging_status = read_AXP2101_charging_status();
    adcOn();
  #endif
    float voltage = SoC->Battery_voltage();
    PRINTLN("Battery_loop: Battery voltage: " + String(voltage) + " V");
 
    if ( hw_info.model    == SOFTRF_MODEL_SKYVIEW &&
        (hw_info.revision == HW_REV_H741_01)) {

      if (voltage > 2.0 && voltage < Battery_cutoff()) {
        if (Battery_cutoff_count > 3) {
          const char* lowBat = "LOW BATTERY";
          LOG_WARN(lowBat);
          ESP32_TFT_fini(lowBat);
          shutdown(lowBat);
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
