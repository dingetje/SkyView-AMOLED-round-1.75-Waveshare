#if defined(AUDIO)

#include "SoundHelper.h"
#include "driver/audio/es8311.h"
#include <Audio.h>
#include <SD_MMC.h>
#include <vector>
#include <string>

//#include "SensorPCF85063.hpp"// RTC

Audio audio; // I2S channel 0 by default
static std::vector<String> fileVector;
//static SensorPCF85063 rtc;

static es8311_handle_t es_handle;

#define EXAMPLE_SAMPLE_RATE 8000
#define DEFAULT_VOICE_VOLUME 80

esp_err_t es8311_codec_init(void) {
  es_handle = es8311_create(0, ES8311_ADDRRES_0);
  ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, "ES8311", "create failed");

  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = EXAMPLE_SAMPLE_RATE * 256,
    .sample_frequency = EXAMPLE_SAMPLE_RATE
  };

  ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_ERROR_CHECK(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency));
  ESP_ERROR_CHECK(es8311_microphone_config(es_handle, false));
  ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, DEFAULT_VOICE_VOLUME, NULL));
  return ESP_OK;
}

String listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.println("Listing directory: " + String(dirname));

  String dirContent = "Listing directory: " + String(dirname) + "\n";

  fs::File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return "Failed to open directory\n";
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return "Not a directory\n";
  }

  fs::File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      String dirName = "  DIR : " + String(file.name());
      Serial.println(dirName);
      dirContent += dirName;
      if (levels) {
        dirContent += listDir(fs, file.path(), levels - 1);
      }
    } else {
      String fileInfo = "  FILE: " + String(file.name()) + "  SIZE: " + String(file.size());
      Serial.println(fileInfo);
      dirContent += fileInfo;
    }
    file = root.openNextFile();
  }
  return dirContent;
}

bool SetupSound()
{
  // start I2S & codec
  Serial.println("starting I2S...");

  // set I2S pins for this board
  if (!audio.setPinout(BCLKPIN,WSPIN,DOPIN,-1,MCLKPIN))
  {
    Serial.println("Failed to set audio pins!");
    return false;
  }
  // DAC init
  if (es8311_codec_init() != ESP_OK)
  {
    Serial.println("Error initializing ES8311 DAC!");
    return false;
  }

  // enable PA
  pinMode(PAPIN, OUTPUT);    // Configure pin 46 (PA) as output
  digitalWrite(PAPIN, HIGH); // Set pin 46 (PA) to HIGH to enable amplifier

  audio.setVolume(21); // default 0...21, weird scale...

  Serial.println("I2S initialized!");
  
  Serial.println("Mounting SD card...");
  if (!SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA))
  {
    Serial.println("SD_MMC setPins failed!");
    return false;
  }
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD card mount failed!");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD_MMC card attached");
    return false;
  }

  Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.println("SD_MMC Card Size: " + String(cardSize) + "MB");
  String dirList = listDir(SD_MMC, "/", 1);
 
//  Serial.println("Starting RTC...");
//  if (!rtc.begin(Wire,0x51,IIC_SDA,IIC_SCL))
//  {
//    Serial.println("Failed to find PCF8563 RTC - check your wiring!");
//    return false;
//  }
//  Serial.println("RTC started!");

  return true;
}

bool play_file(char *filename, int volume)
{
  Serial.println("play_file: '" + String(filename) + "'");
  if (!SD_MMC.exists(filename)) {
    Serial.println("play_file: '" + String(filename) + "' does not exist");
    return false;
  }
  es8311_voice_volume_set(es_handle, volume, NULL);
  audio.connecttoFS(SD_MMC,filename);
  return true;
}

void SoundLoop()
{
  // next file in the vector?
  if (audio.isRunning())
  {
      audio.loop();
      vTaskDelay(1);
      return;
  }
  if (!fileVector.empty())
  {
    // pop next filename from the playlist
    String fileName = fileVector.front();
    fileVector.erase(fileVector.begin());
    play_file((char*)fileName.c_str(),DEFAULT_VOICE_VOLUME);
  }
}

/* add a file to the audio queue, file already checked for existance on SD */
void add_file(char* filePath)
{
  fileVector.push_back(String(filePath));
}

/* if defined, print info about audio process
void audio_info(const char *info)
{
//    RTC_DateTime datetime = rtc.getDateTime();
//    Serial.print(datetime.year);
//    Serial.print("/");
//    Serial.print(datetime.month);
//    Serial.print("/");
//    Serial.print(datetime.day);
//    Serial.print(" ");
//    Serial.print(datetime.hour);
//    Serial.print(":");
//    Serial.print(datetime.minute);
//    Serial.print(":");
//    Serial.print(datetime.second);
//    Serial.print(" | ");
    Serial.println(info);
}
*/

#endif
