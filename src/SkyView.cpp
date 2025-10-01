/*
 * SkyView(.ino) firmware
 * Copyright (C) 2019-2021 Linar Yusupov
 *
 * This firmware is essential part of the SoftRF project.
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Credits:
 *   Arduino core for ESP8266 is developed/supported by ESP8266 Community (support-esp8266@esp8266.com)
 *   Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 *   TinyGPS++ and PString Libraries are developed by Mikal Hart
 *   Arduino core for ESP32 is developed/supported by Hristo Gochkov
 *   jQuery library is developed by JS Foundation
 *   BCM2835 C library is developed by Mike McCauley
 *   GxEPD2 library is developed by Jean-Marc Zingg
 *   Adafruit GFX library is developed by Adafruit Industries
 *   GDL90 decoder is developed by Ryan David
 *   Sqlite3 Arduino library for ESP32 is developed by Arundale Ramanathan
 *   uCDB library is developed by Ioulianos Kakoulidis
 *   FLN/OGN aircrafts data is courtesy of FlarmNet/GliderNet
 *   Adafruit SSD1306 library is developed by Adafruit Industries
 *   ESP32 I2S WAV player example is developed by Tuan Nha
 *   AceButton library is developed by Brian Park
 *   Flashrom library is part of the flashrom.org project
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
#include "Arduino.h"
#include "esp_system.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "EPDHelper.h"
#include "TrafficHelper.h"
#include "WiFiHelper.h"
#include "WebHelper.h"
#include "BatteryHelper.h"
#include "GDL90Helper.h"
#include "TFTHelper.h"
#include "TouchHelper.h"
#include "BuddyHelper.h"

#if defined(ES8311_AUDIO)
#include "SoundHelper.h"
#endif

#if defined(WAVESHARE_AMOLED_1_75)
#include "RTCHelper.h"
#endif

#include "SPIFFS.h"
#include "SkyView.h"
// #include "TFTHelper.h"
#if defined(CPU_float)
extern "C" {
  #include "esp_pm.h"
}
#endif

// If you want use SD_MMC (ESP) or other FileSystems
#include <SD_MMC.h>
#if defined(DEBUG_MODE)
// define DEBUGLOG_ENABLE_FILE_LOGGER to enable file logger
#define DEBUGLOG_ENABLE_FILE_LOGGER
#endif
#include <DebugLog.h>

hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_SKYVIEW,
  .revision = HW_REV_UNKNOWN,
  .soc      = SOC_NONE,
  .display  = DISPLAY_NONE
};

static bool SDCard_Mounted = false;
bool SPIFFS_is_mounted = false;

/* Poll input source(s) */
void Input_loop() {
  switch (settings->protocol)
  {
  case PROTOCOL_GDL90:
    GDL90_loop();
    break;
  case PROTOCOL_NMEA:
  default:
    NMEA_loop();
    break;
  }
}

String listDir(fs::FS &fs, const char *dirname, uint8_t levels) 
{
  Serial.println("Listing directory: " + String(dirname));

  String dirContent = "Listing directory: " + String(dirname) + "\n";

  fs::File root = fs.open(dirname);
  if (!root) 
  {
    Serial.println("Failed to open directory");
    return "Failed to open directory\n";
  }
  if (!root.isDirectory()) 
  {
    Serial.println("Not a directory");
    return "Not a directory\n";
  }

  fs::File file = root.openNextFile();
  while (file) 
  {
    if (file.isDirectory()) 
    {
      String dirName = "  DIR : " + String(file.name());
      Serial.println(dirName);
      dirContent += dirName;
      if (levels) 
      {
        dirContent += listDir(fs, file.path(), levels - 1);
      }
    }
    else 
    {
      String fileInfo = "  FILE: " + String(file.name()) + "  SIZE: " + String(file.size());
      Serial.println(fileInfo);
      dirContent += fileInfo;
    }
    file = root.openNextFile();
  }
  return dirContent;
}

bool MountSDCard()
{
  bool result = false;
  PRINTLN("Mounting SD card...");
  if (!SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA))
  {
    PRINTLN("[ERROR] SD_MMC setPins failed!");
  }
  else
  {
    if (!SD_MMC.begin("/sdcard", true)) 
    {
      PRINTLN("[ERROR] SD_MMC card mount failed!");
    }
    else
    {
      uint8_t cardType = SD_MMC.cardType();
      if (cardType == CARD_NONE) 
      {
        PRINTLN("[ERROR] No SD_MMC card attached");
      }
      else
      {
        PRINT("SD_MMC Card Type: ");
        switch(cardType)
        {
          case CARD_MMC:  PRINTLN("MMC"); break;
          case CARD_SD:   PRINTLN("SDSC"); break;
          case CARD_SDHC: PRINTLN("SDHC"); break;
          default:        PRINTLN("UNKNOWN"); break;
        }
        uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
        PRINTLN("SD_MMC Card Size: " + String(cardSize) + "MB");
        listDir(SD_MMC, "/", 1);
        PRINTLN("SD_MMC fileSystem initialization success");

#if defined(DEBUG_MODE)
        String filename = "/logs/log_" + String(__TIME__) + ".txt";
        // Set file system to save every log automatically
        LOG_ATTACH_FS_AUTO(SD_MMC, filename, FILE_WRITE);  // overwrite file
        LOG_FILE_SET_LEVEL(DebugLogLevel::LVL_INFO);

        // The default log_leval is DebugLogLevel::LVL_INFO
        // The default file_leval is DebugLogLevel::LVL_ERROR
        // 0: NONE, 1: ERROR, 2: WARN, 3: INFO, 4: DEBUG, 5: TRACE
        PRINTLN_FILE("current file log level is", (int)LOG_FILE_GET_LEVEL());
#endif
        result = true;
      }
    }
  }
  SDCard_Mounted = result;
  return result;
}

void setup()
{
  uint8_t mac[6];

  Serial.begin(SERIAL_OUT_BR); 
  delay(3000); // allow to connect terminal

  Serial.println();
  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  Serial.println();
  Serial.print(F(SKYVIEW_IDENT));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SKYVIEW_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.print(F(" FLASH ID: "));
  Serial.println(String(SoC->getFlashId(), HEX));
  Serial.print(F(" PSRAM FOUND: "));
  Serial.println(ESP.getPsramSize() / 1024);
  Serial.print(F(" FLASH SIZE: "));
  Serial.print(ESP.getFlashChipSize() / 1024);
  Serial.println(F(" KB"));

   // Options: ESP_MAC_WIFI_STA, ESP_MAC_WIFI_SOFTAP, ESP_MAC_BT, ESP_MAC_ETH
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("WiFi MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  esp_read_mac(mac, ESP_MAC_BT);
  Serial.printf("BLE MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);


  // load settings
  EEPROM_setup();
  if (settings == NULL || SoC == NULL || SoC->Bluetooth == NULL) 
  {
    PRINTLN("Error: Null pointer detected!");
    return;
  }
#if defined(USE_TFT)
  TFT_setup();
  hw_info.display = DISPLAY_TFT;
#endif /* USE_TFT */
  if (hw_info.display != DISPLAY_NONE) 
  {
    Serial.println(F(" done."));
  }
  else
  {
    Serial.println(F(" failed!"));
  }

  // BuddyList.txt
  if (!SPIFFS.begin(true)) 
  {
    SPIFFS_is_mounted = false;
    PRINTLN("SPIFFS Mount Failed");
  }
  else
  {
    SPIFFS_is_mounted = true;
    PRINTLN("SPIFFS mounted successfully");
    PRINT("SPIFFS Total space: ");
    PRINTLN(SPIFFS.totalBytes());
    PRINT("SPIFFS Used space: ");
    PRINTLN(SPIFFS.usedBytes());

    // List all files in SPIFFS
    PRINTLN("Dumping SPIFFS files:");
    fs::File root = SPIFFS.open("/");
    fs::File file = root.openNextFile(); 
    while(file)
    {
        PRINT("FILE: ");
        PRINTLN(file.name()); 
        file = root.openNextFile();
    }
    BuddyManager::readBuddyList("/buddylist.txt");  // Read buddy list from SPIFFS
    // Print buddy list to serial
    BuddyManager::printBuddyList();
  }

  PRINTLN(F("Copyright (C) 2019-2025 Linar Yusupov. All rights reserved."));
  Serial.flush();

  //temporary settings
 /* settings->adapter       = ADAPTER_TTGO_T5S;
  settings->connection      = CON_BLUETOOTH_LE;
  settings->bridge          = BRIDGE_NONE;
  settings->baudrate        = B38400;
  settings->protocol        = PROTOCOL_NMEA;
  settings->orientation     = DIRECTION_NORTH_UP;
  settings->units           = UNITS_METRIC;
  settings->vmode           = VIEW_MODE_RADAR;
  settings->zoom            = ZOOM_MEDIUM;
  settings->adb             = DB_NONE;
  settings->idpref          = ID_REG;
  settings->voice           = VOICE_OFF;
  settings->compass          = ANTI_GHOSTING_OFF;
  settings->filter          = TRAFFIC_FILTER_500M;
  settings->power_save      = POWER_SAVE_WIFI;
  settings->team            = 0x46BCDC;
*/
  Battery_setup();
#if defined(BUTTONS)
  SoC->Button_setup();
#endif /* BUTTONS */

#if defined(WAVESHARE_AMOLED_1_75)
  RTC_Setup();
#endif
#if defined(ES8311_AUDIO)
  PRINTLN("Intializing I2S and ES8311 audio module... ");
  SetupSound();
#endif /* ES8311_AUDIO */

  switch (settings->protocol)
  {
  case PROTOCOL_GDL90:
    GDL90_setup();
    break;
  case PROTOCOL_NMEA:
  default:
    NMEA_setup();
    break;
  }

  /* If a Dongle is connected - try to wake it up */
  if (settings->connection == CON_SERIAL &&
      settings->protocol   == PROTOCOL_NMEA) 
  {
    SerialInput.write("$PSRFC,?*47\r\n");
    SerialInput.flush();
  }

#if defined(CPU_float)
  esp_pm_config_esp32s3_t pm_config = {
    .max_freq_mhz = 240,
    .min_freq_mhz = 80,
    .light_sleep_enable = false
  };
  esp_pm_configure(&pm_config);
#endif /* CPU_float */

  WiFi_setup();

#if defined(DB)
  if (SoC->DB_init())
  {
    PRINTLN("OGN database initialized successfully");
#if defined (DB_DEBUG)
    char ogn_name[64];
    char ogn_additional[64];
    // do a test query with a known ID
    if (SoC->DB_query(DB_OGN, 0x484BBC, ogn_name, sizeof(ogn_name), ogn_additional, sizeof(ogn_additional)))
    {
      PRINT("OGN DB query test: 0x484BBC = ");
      PRINT(ogn_name);
      PRINT(" (");
      PRINT(ogn_additional);
      PRINTLN(")");
    }
    else
    {
      PRINTLN("OGN database query failed!");
    }
#endif
  }
  else
  {
    PRINTLN("OGN database initialization failed");
  }
#endif
#if defined(AUDIO)
  SoC->TTS((char *)"POST");
#endif
  Web_setup();
  Traffic_setup();
#if defined(AMOLED)
  Touch_setup();
#endif
  print_wakeup_reason();
  SoC->WDT_setup();
}

void loop()
{
  if (SoC->Bluetooth) {
    bool wdt_status = loopTaskWDTEnabled;

    if (wdt_status) {
      disableLoopWDT();
    }
    SoC->Bluetooth->loop();

    if (wdt_status) {
      enableLoopWDT();
    }
  }

  Input_loop();

  Traffic_loop();
#if defined(USE_EPAPER)
  EPD_loop();
#elif defined(USE_TFT)
  TFT_loop(); 
#endif /* USE_EPAPER */
  Traffic_ClearExpired();

  WiFi_loop();

#if defined(ES8311_AUDIO)
  SoundLoop();
#endif
  // Handle Web
  Web_loop();
#if defined(BUTTONS)
  SoC->Button_loop();
#endif
  Battery_loop();
  RTC_Loop();
}

void shutdown(const char *msg)
{
  PRINTLN("Shutdown '" + String(msg) + "'");
  SoC->WDT_fini();
  /* If a Dongle is connected - try to shut it down */
  if (settings->connection == CON_SERIAL &&
      settings->protocol   == PROTOCOL_NMEA) {
    SerialInput.write("$PSRFC,OFF*37\r\n");
    SerialInput.flush();
  }

  if (SoC->Bluetooth) {
    SoC->Bluetooth->fini();
  }

  Web_fini();
#if defined(DB)
  SoC->DB_fini();
#endif /* DB */
  WiFi_fini();
#if defined(USE_EPAPER)
  EPD_fini(msg);
#elif defined(USE_TFT)
  ESP32_TFT_fini(msg);
#endif /* USE_EPAPER */
#if defined(BUTTONS)
  SoC->Button_fini();
#endif /* BUTTONS */
  ESP32_fini();
}
