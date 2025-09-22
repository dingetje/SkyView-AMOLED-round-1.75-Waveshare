#if defined(AUDIO)

#include "driver/audio/es8311.h"
#include <Audio.h>
#include <vector>
#include <string>
#include "SoundHelper.h"
#include <DebugLog.h>

Audio audio; // I2S channel 0 by default
static std::vector<String> fileVector;

static es8311_handle_t es_handle;
static unsigned long LastSoundLoop = 0;
static bool bSoundInitialized = false;

#define EXAMPLE_SAMPLE_RATE 8000
// too high volume leads to distortion on the tiny speaker
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

bool SetupSound()
{
  bSoundInitialized = false;
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    PRINTLN("[ERROR] SetupSound skipped, no SD_MMC card found, required for Audio files");
    return false;
  }

  // start I2S & codec
  Serial.println("starting I2S...");

  // set I2S pins for this board
  if (!audio.setPinout(BCLKPIN,WSPIN,DOPIN,-1,MCLKPIN))
  {
    PRINTLN("[ERROR] Failed to set audio pins!");
    return false;
  }
  // DAC init
  if (es8311_codec_init() != ESP_OK)
  {
    PRINTLN("Error initializing ES8311 DAC!");
    return false;
  }
  // set fade in/fade out
  // es8311_voice_fade(es_handle, es8311_fade_t::ES8311_FADE_4LRCK);

  // enable PA
  pinMode(PAPIN, OUTPUT);    // Configure pin 46 (PA) as output
  digitalWrite(PAPIN, HIGH); // Set pin 46 (PA) to HIGH to enable amplifier

  audio.setVolume(21);  // default 0...21, see wiki https://github.com/schreibfaul1/ESP32-audioI2S/wiki
  audio.forceMono(true);// Waveshare board has only one speaker, so set to mono

  PRINTLN("I2S and ES8311 DAC initialized!");
   
  bSoundInitialized = true;
  return true;
}

bool play_file(char *filename, int volume)
{
  if (bSoundInitialized)
  {
    PRINTLN("play_file: '" + String(filename) + "'");
    if (!SD_MMC.exists(filename)) {
      Serial.println("play_file: '" + String(filename) + "' does not exist");
      return false;
    }
    es8311_voice_volume_set(es_handle, volume, NULL);
    audio.connecttoFS(SD_MMC,filename);
  }
  return true;
}

void SoundLoop()
{
  // if not initialized, do nothing
  if (!bSoundInitialized)
  {
    return;
  }

  unsigned long now = millis();
  if (audio.isRunning())
  {
      audio.loop();
      vTaskDelay(1);
      return;
  }
  // next file in the vector?
  if (!fileVector.empty())
  {
    Serial.printf("SoundLoop: %u ms\r\n", now - LastSoundLoop);
    LastSoundLoop = now;
    // pop next filename from the playlist
    String fileName = fileVector.front();
    fileVector.erase(fileVector.begin());
    play_file((char*)fileName.c_str(),DEFAULT_VOICE_VOLUME);
  }
}

/* add a file to the audio queue, file already checked for existance on SD */
void add_file(const char* filePath)
{
  // only makes sense if sound was correctly initialized
  if (bSoundInitialized)
  {
    fileVector.push_back(String(filePath));
  }
}

/* if defined, print info about audio process */
//void audio_info(const char *info)
//{
//    Serial.println(info);
//}

bool IsSoundInitialized()
{
  return bSoundInitialized;
}

#endif
