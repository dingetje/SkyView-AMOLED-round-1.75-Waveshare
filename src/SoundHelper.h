#ifndef _SOUNDHELPER_H
#define _SOUNDHELPER_H

#include "esp_check.h"
#include "../pins_config.h"
//#include "AudioTools.h"
//#include "AudioTools/AudioLibs/AudioBoardStream.h"
#include <SD_MMC.h>

// function to play a file, returns false if the file does not exist
// volume is between 0.0 and 1.0, default is 0.7
// much higher values may cause distortion
//bool play_file(char *filename, float volume = 0.7f);

void add_file(char* filePath);

extern void SoundLoop();
extern bool SetupSound();

#endif
