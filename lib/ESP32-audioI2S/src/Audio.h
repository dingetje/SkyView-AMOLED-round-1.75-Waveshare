/*
 * Audio.h
 *
 *  Created on: Oct 28,2018
 *  Updated on: Nov 22,2022
 *      Author: Wolle (schreibfaul1)
 * 
 * Butchered by Antoine Megens for Skyview TTS
 */

//#define SDFATFS_USED  // activate for SdFat


#pragma once
#pragma GCC optimize ("Ofast")
#include <vector>
#include <Arduino.h>
#include <libb64/cencode.h>
#include <esp32-hal-log.h>

#include <vector>
#include <driver/i2s.h>

#ifndef AUDIO_NO_SD_FS
#include <SPI.h>
#ifdef SDFATFS_USED
#include <SdFat.h>  // https://github.com/greiman/SdFat
#else
#include <SD_MMC.h>
#endif // SDFATFS_USED
#endif

using namespace std;

extern __attribute__((weak)) void audio_info(const char *);
extern __attribute__((weak)) void audio_process_i2s(uint32_t* sample, bool *continueI2S); // record audiodata or send via BT

#define AUDIO_INFO(...) {char buff[512 + 64]; snprintf(buff,sizeof(buff),__VA_ARGS__); if(audio_info) audio_info(buff);}

/* these are data structures to process wav file */
typedef enum headerState_e {
    HEADER_RIFF, HEADER_FMT, HEADER_DATA, DATA
} headerState_t;

typedef struct wavRiff_s {
    uint32_t chunkID;
    uint32_t chunkSize;
    uint32_t format;
} wavRiff_t;

typedef struct wavProperties_s {
    uint32_t chunkID;
    uint32_t chunkSize;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
} wavProperties_t;

//----------------------------------------------------------------------------------------------------------------------

class Audio {

public:
    Audio(bool internalDAC = false, uint8_t channelEnabled = 3, uint8_t i2sPort = I2S_NUM_0); // #99
    ~Audio();
#ifndef AUDIO_NO_SD_FS
    bool connecttoFS(fs::FS &fs, const String path);
    bool connecttoSD(const String path);
    bool playSDFileList(std::vector<String>& fileList);
    void resetPlayList();
#endif // AUDIO_NO_SD_FS
    bool setPinout(uint8_t BCLK, uint8_t LRC, uint8_t DOUT, int8_t DIN = I2S_PIN_NO_CHANGE, int8_t MCK = I2S_PIN_NO_CHANGE);
    bool isRunning() {return m_f_running;}
    void loop();
    uint32_t stopSong();
    uint32_t getFileSize();
    uint32_t getFilePos();
    uint32_t getSampleRate();
    uint8_t  getBitsPerSample();
    uint8_t  getChannels();
    uint32_t getAudioFileDuration();
    uint32_t getAudioCurrentTime();
    void forceMono(bool mono);
    void setTone(int8_t gainLowPass, int8_t gainBandPass, int8_t gainHighPass);
    int getCodec() {return m_codec;}
    const char *getCodecname() {return codecname[m_codec];}
    inline uint8_t getDatamode(){return m_datamode;}
    void playWavFileTask();

private:

    #ifndef ESP_ARDUINO_VERSION_VAL
        #define ESP_ARDUINO_VERSION_MAJOR 0
        #define ESP_ARDUINO_VERSION_MINOR 0
        #define ESP_ARDUINO_VERSION_PATCH 0
    #endif

    void setDefaults(); // free buffers and set defaults
    void showCodecParams();
    void compute_audioCurrentTime(int bd);
    size_t readAudioHeader(uint32_t bytes);
    int  read_WAV_Header(size_t len);
    bool setSampleRate(uint32_t hz);
    bool setBitsPerSample(int bits);
    bool setChannels(int channels);
    bool initializeDecoder();
    inline void setDatamode(uint8_t dm){m_datamode=dm;}

    int read4bytes(File file, uint32_t *chunkId);
    int readRiff(File file, wavRiff_t *wavRiff);
    int readProps(File file, wavProperties_t *wavProps);

    //++++ implement several function with respect to the index of string ++++
    void trim(char *s) 
    {
        //fb   trim in place
        char *pe;
        char *p = s;
        while ( isspace(*p) ) p++; //left
        pe = p; //right
        while ( *pe != '\0' ) pe++;
        do {
            pe--;
        } while ( (pe > p) && isspace(*pe) );
        if (p == s) {
            *++pe = '\0';
        } else {  //move
            while ( p <= pe ) *s++ = *p++;
            *s = '\0';
        }
    }

    bool startsWith (const char* base, const char* str) 
    {
        //fb
        char c;
        while ( (c = *str++) != '\0' )
          if (c != *base++) return false;
        return true;
    }

    bool endsWith (const char* base, const char* str) 
    {
        //fb
        int slen = strlen(str) - 1;
        const char *p = base + strlen(base) - 1;
        while(p > base && isspace(*p)) p--;  // rtrim
        p -= slen;
        if (p < base) return false;
        return (strncmp(p, str, slen) == 0);
    }

    int indexOf (const char* base, const char* str, int startIndex = 0) 
    {
        //fb
        const char *p = base;
        for (; startIndex > 0; startIndex--)
            if (*p++ == '\0') return -1;
        char* pos = strstr(p, str);
        if (pos == nullptr) return -1;
        return pos - base;
    }

    int indexOf (const char* base, char ch, int startIndex = 0) 
    {
        //fb
        const char *p = base;
        for (; startIndex > 0; startIndex--)
            if (*p++ == '\0') return -1;
        char *pos = strchr(p, ch);
        if (pos == nullptr) return -1;
        return pos - base;
    }

    int lastIndexOf(const char* haystack, const char* needle) 
    {
        //fb
        int nlen = strlen(needle);
        if (nlen == 0) return -1;
        const char *p = haystack - nlen + strlen(haystack);
        while (p >= haystack) {
          int i = 0;
          while (needle[i] == p[i])
            if (++i == nlen) return p - haystack;
          p--;
        }
        return -1;
    }

    int lastIndexOf(const char* haystack, const char needle) 
    {
        //fb
        const char *p = strrchr(haystack, needle);
        return (p ? p - haystack : -1);
    }

    int specialIndexOf (uint8_t* base, const char* str, int baselen, bool exact = false)
    {
        int result;  // seek for str in buffer or in header up to baselen, not nullterninated
        if (strlen(str) > baselen) return -1; // if exact == true seekstr in buffer must have "\0" at the end
        for (int i = 0; i < baselen - strlen(str); i++)
        {
            result = i;
            for (int j = 0; j < strlen(str) + exact; j++)
            {
                if (*(base + i + j) != *(str + j))
                {
                    result = -1;
                    break;
                }
            }
            if (result >= 0) break;
        }
        return result;
    }

private:
    const char *codecname[3] = {"unknown", "WAV", "RAW" };
    enum : int { AUDIO_NONE, AUDIO_DATA, AUDIO_LOCALFILE };
    enum : int { APLL_AUTO = -1, APLL_ENABLE = 1, APLL_DISABLE = 0 };
    enum : int { EXTERNAL_I2S = 0, INTERNAL_DAC = 1, INTERNAL_PDM = 2 };
    enum : int { CODEC_NONE = 0, CODEC_WAV = 1, CODEC_RAW = 2};
    typedef enum { LEFTCHANNEL=0, RIGHTCHANNEL=1 } SampleIndex;

#ifndef AUDIO_NO_SD_FS
    File                    audiofile;    // @suppress("Abstract class cannot be instantiated")
#endif  // AUDIO_NO_SD_FS
    i2s_config_t            m_i2s_config = {}; // stores values for I2S driver
    i2s_pin_config_t        m_pin_config = {};

    std::vector<String>*    m_fileList;   // list of files for connecttoFS(fs, fileList)
    int                     m_fileListIndex = 0;
    unsigned long           m_fileStartMilis;
    unsigned long           m_fileEndMilis;
    uint32_t                m_sampleRate=16000;
    int                     m_readbytes = 0;                // bytes read
    uint8_t                 m_bitsPerSample = 16;           // bitsPerSample
    uint8_t                 m_avr_bitrate;                  // average bitrate
    uint8_t                 m_channels = 2;
    uint8_t                 m_i2s_num = I2S_NUM_0;          // I2S_NUM_0 or I2S_NUM_1
    uint8_t                 m_codec = CODEC_NONE;           //
    uint8_t                 m_headerBuff[44];               // WAV header
    uint16_t                m_datamode = 0;                 // State machine
    bool                    m_f_running = false;
    bool                    m_f_forceMono = true;
    bool                    m_f_firstCall = false;          // InitSequence for processLocalFile
    bool                    m_f_internalDAC = false;        // false: output vis I2S, true output via internal DAC
    bool                    m_f_Log = false;                // set in platformio.ini  -DAUDIO_LOG and -DCORE_DEBUG_LEVEL=3 or 4
    uint8_t                 m_f_channelEnabled = 3;         // internal DAC, both channels
    size_t                  m_i2s_bytesWritten = 0;         // set in i2s_write() but not used
    size_t                  m_file_size = 0;                // size of the file
};

//----------------------------------------------------------------------------------------------------------------------
