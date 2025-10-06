#ifndef _SOUNDHELPER_H
#define _SOUNDHELPER_H

#if defined(ES8311_AUDIO)

#include "esp_check.h"
#include "../pins_config.h"
#include <SD_MMC.h>

void add_file(const char *filePath);

extern void SoundLoop();
extern bool SetupSound();
extern bool IsSoundInitialized();
extern void PlayFileList(bool alarm);
#endif
#endif
