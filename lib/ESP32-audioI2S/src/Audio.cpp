/*
 * Audio.cpp
 *
 *  Created on: Oct 26.2018
 *
 *  Version 2.0.7
 *  Updated on: Nov 22.2022
 *      Author: Wolle (schreibfaul1)
 *
 *  Stripped down and modified for Waveshare Skyview audio TTS
 *  by Antoine Megens, Oct 2025
 * 
 */
#include "Audio.h"
#include <soc/soc.h>
#include <soc/io_mux_reg.h>

#ifndef DISABLE_PROFILER
#include <Profiler.h>
#endif

#ifndef AUDIO_NO_SD_FS
#ifdef SDFATFS_USED
fs::SDFATFS SD_SDFAT;
#endif
#endif // AUDIO_NO_SD_FS

#define CCCC(c1, c2, c3, c4)    ((c4 << 24) | (c3 << 16) | (c2 << 8) | c1)

void dumpBuffer(const uint8_t* buffer, size_t length) 
{
  const size_t bytesPerLine = 16;
  for (size_t i = 0; i < length; i += bytesPerLine) 
  {
    // Print offset
    Serial.printf("%04X  ", (unsigned int)i);

    // Print hex values
    for (size_t j = 0; j < bytesPerLine; ++j) 
    {
      if (i + j < length)
        Serial.printf("%02X ", buffer[i + j]);
      else
        Serial.print("   ");
    }
    Serial.flush();
    Serial.print(" ");

    // Print ASCII representation
    for (size_t j = 0; j < bytesPerLine; ++j) 
    {
      if (i + j < length) 
      {
        char c = buffer[i + j];
        Serial.print(isPrintable(c) ? c : '.');
      }
    }
    Serial.flush();
    Serial.println();
    Serial.flush();
    vTaskDelay(1);
  }
}

//---------------------------------------------------------------------------------------------------------------------
Audio::Audio(bool internalDAC /* = false */, uint8_t channelEnabled /* = I2S_DAC_CHANNEL_BOTH_EN */, uint8_t i2sPort) 
{
    //    build-in-DAC works only with ESP32 (ESP32-S3 has no build-in-DAC)
    //    build-in-DAC last working Arduino Version: 2.0.0-RC2
    //    possible values for channelEnabled are:
    //    I2S_DAC_CHANNEL_DISABLE  = 0,     Disable I2S built-in DAC signals
    //    I2S_DAC_CHANNEL_RIGHT_EN = 1,     Enable I2S built-in DAC right channel, maps to DAC channel 1 on GPIO25
    //    I2S_DAC_CHANNEL_LEFT_EN  = 2,     Enable I2S built-in DAC left  channel, maps to DAC channel 2 on GPIO26
    //    I2S_DAC_CHANNEL_BOTH_EN  = 0x3,   Enable both of the I2S built-in DAC channels.
    //    I2S_DAC_CHANNEL_MAX      = 0x4,   I2S built-in DAC mode max index
#ifdef AUDIO_LOG
    m_f_Log = true;
#endif

    m_f_channelEnabled = channelEnabled;
    m_f_internalDAC = internalDAC;
    //i2s configuration
    m_i2s_num = i2sPort; // i2s port number
    m_i2s_config.sample_rate          = 22050;
    m_i2s_config.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    m_i2s_config.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    m_i2s_config.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1; // interrupt priority
    m_i2s_config.dma_buf_count        = 4;
    m_i2s_config.dma_buf_len          = 512; // 1024 is max.
    m_i2s_config.use_apll             = APLL_DISABLE; // must be disabled in V2.0.1-RC1
    m_i2s_config.tx_desc_auto_clear   = true;   // new in V1.0.1
    m_i2s_config.fixed_mclk           = I2S_PIN_NO_CHANGE; // set in setPinout()

    if (internalDAC)  
    {

        #ifdef CONFIG_IDF_TARGET_ESP32  // ESP32S3 has no DAC

            log_i("internal DAC");
            m_i2s_config.mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN );

            #if ESP_ARDUINO_VERSION_MAJOR >= 2
                #if ESP_ARDUINO_VERSION_PATCH  == 0
                    m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S); // vers == 2.0.0
                #else
                    // V2.0.1 ... V2.0.4 will not work
                    m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_MSB); // vers >= 2.0.5
                #endif
            #else
                m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB);
            #endif

            i2s_driver_install((i2s_port_t)m_i2s_num, &m_i2s_config, 0, NULL);
            i2s_set_dac_mode((i2s_dac_mode_t)m_f_channelEnabled);
            if(m_f_channelEnabled != I2S_DAC_CHANNEL_BOTH_EN) {
                m_f_forceMono = true;
            }

        #endif

    }
    else 
    {
        m_i2s_config.mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);

        #if ESP_ARDUINO_VERSION_MAJOR >= 2
            m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S); // Arduino vers. > 2.0.0
        #else
            m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
        #endif

        if (i2s_driver_install((i2s_port_t)m_i2s_num, &m_i2s_config, 0, NULL) != ESP_OK) 
        {
            log_e("i2s_driver_install failed");
        }
        m_f_forceMono = true;
    }

    i2s_zero_dma_buffer((i2s_port_t) m_i2s_num);
}

#define I2S_BUFFER_SIZE 512             // fixed buffer for AudioTask
#define MONO_BLOCK_SIZE 256             // Half of I2S buffer
uint8_t mono_block[MONO_BLOCK_SIZE];    // 256 bytes raw data
uint16_t stereo_block[MONO_BLOCK_SIZE]; // 512 bytes = 256 stereo samples
// Define a task handle and initialize it to NULL
static TaskHandle_t task_handle = NULL;

// parameter is the Audio class itself
void audioTask(void *parameter)
{
    Audio* pAudio = (Audio*) parameter;
    pAudio->playWavFileTask();
}

// implementation of the AudioTask, this replaces the too slow 
// Audio::loop() call that caused way too much overhead for TTS
// This is a much tighter loop and has almost no overhead
void Audio::playWavFileTask()
{
    while(true)
    {
        if (!isRunning())
        {
            vTaskDelay(50);
        }
        else if (getDatamode() == AUDIO_LOCALFILE && audiofile.available())
        {
            // WAV header already parsed
            size_t bytes_read, bytes_written;
            // fixed data rate is fine, all audio files are the same
            // i2s_set_sample_rates((i2s_port_t)0, 22050);
            while (audiofile.available()) 
            {
                // convert mono data to stereo
                bytes_read = audiofile.read(mono_block, MONO_BLOCK_SIZE);
                size_t samples = bytes_read / 2;

                for (size_t i = 0; i < samples; ++i) 
                {
                    uint16_t sample = mono_block[2 * i] | (mono_block[2 * i + 1] << 8);
                    stereo_block[i * 2] = sample;     // Left
                    stereo_block[i * 2 + 1] = sample; // Right
                }
                i2s_write((i2s_port_t) 0, stereo_block, samples * 4, &bytes_written, 100);
                vTaskDelay(1);  // Yield to avoid watchdog
            }
            stopSong(); // will start next file if one in the play list
            vTaskDelay(200); // pauze between words
        }
    }
}
//---------------------------------------------------------------------------------------------------------------------
Audio::~Audio() 
{
    //I2Sstop(m_i2s_num);
    //InBuff.~AudioBuffer(); #215 the AudioBuffer is automatically destroyed by the destructor
    setDefaults();
#ifndef AUDIO_LIB_LEAN_AND_MEAN
    if(m_playlistBuff) {free(m_playlistBuff); m_playlistBuff = NULL;}
#endif
    i2s_driver_uninstall((i2s_port_t)m_i2s_num); // #215 free I2S buffer
#ifndef AUDIO_LIB_LEAN_AND_MEAN
    if(m_chbuf) {free(m_chbuf); m_chbuf = NULL;}
#endif
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setDefaults() 
{
//    Serial.printf("AudioI2S: chbufSize = %d\r\n", m_chbufSize);
    stopSong();

    m_f_firstCall = true;                                   // InitSequence for processWebstream and processLokalFile
    m_f_running = false;
    m_codec = CODEC_NONE;
    m_datamode = AUDIO_NONE;
    m_avr_bitrate = 0;                                      // the same as m_bitrate if CBR, median if VBR
    m_channels = 2;                                         // assume stereo #209
    m_file_size = 0;
}
//---------------------------------------------------------------------------------------------------------------------
#ifndef AUDIO_NO_SD_FS
bool Audio::connecttoSD(const String path) 
{
    return connecttoFS(SD_MMC, path);
}

bool Audio::playSDFileList(std::vector<String>& playlist)
{
#ifndef DISABLE_PROFILER    
    profiler_t profiler(
        (String("Time spent in ") + 
        String(__FUNCTION__) + 
        String("()")).c_str());
#endif
    if(playlist.size() == 0) return false;
    // keep a reference to the play list
    m_fileList = &playlist;
    // only do this is not busy with the file list
    if (!m_f_running)
    {
        m_fileListIndex = 0;
        m_fileEndMilis = 0;
        setDefaults();
        String fileName = m_fileList->at(0);
        connecttoFS(SD_MMC, fileName.c_str());
    }
 //   Serial.printf("Audio::playSDFileList: %d files in playlist\r\n", m_fileList->size());
//    Serial.flush();
    return true;
}

void Audio::resetPlayList()
{
    m_f_running = false; // has to be set first to avoid next file in playlist in stopSong!
    stopSong();
    vTaskDelay(50);
    m_fileListIndex = 0;
    m_fileEndMilis = 0;
    if (!m_fileList->empty())
    {
        String fileName = m_fileList->at(0);
        Serial.printf("Audio::resetPlayList: %d files in playlist, resume playing with: '%s'\r\n", m_fileList->size(), fileName.c_str());
        Serial.flush();
        connecttoFS(SD_MMC, fileName.c_str());
    }
}

//---------------------------------------------------------------------------------------------------------------------
bool Audio::connecttoFS(fs::FS &fs, String path) 
{
#ifndef DISABLE_PROFILER
    profiler_t profiler(
        (String("Time spent in ") + 
        String(__FUNCTION__) + 
        String("()")).c_str());
#endif
    if (path.length() == 0) 
    {
        AUDIO_INFO("File path is empty");
        return false;
    }

    m_fileStartMilis = millis();
    if (m_fileEndMilis > 0) 
    {
        Serial.printf("Elapsed time since last file end: %u ms\r\n", m_fileStartMilis - m_fileEndMilis);
    }

    if(!path.startsWith("/"))
    {
        path = "/" + path;
    }

    AUDIO_INFO("Reading file: '%s'", path.c_str()); vTaskDelay(2);
    if(fs.exists(path)) 
    {
        audiofile = fs.open(path); // #86
    }

    if(!audiofile) 
    {
        if(audio_info) 
        {
            vTaskDelay(2); 
            audio_info("Failed to open file for reading");
        }
        return false;
    }

    String afn = path;
    afn.toLowerCase();
    if(afn.endsWith(".wav"))  m_codec = CODEC_WAV;
    if(afn.endsWith(".raw"))  m_codec = CODEC_RAW; // not sure what this format is, raw PCM data?

    if(m_codec == CODEC_NONE)
    {
        AUDIO_INFO("The '%s' file format is not supported", afn);
        audiofile.close();
        return false;
    }

    bool ret = initializeDecoder();
    if(ret)
    {
        setDatamode(AUDIO_LOCALFILE);
        m_file_size = audiofile.size();//TEST loop

        // first time here?
        if (task_handle == NULL)
        {
            // Create the task, and pass the task handle as last parameter
            // to xTaskCreate (pointer):
            xTaskCreate(
                audioTask,     // Function that should be called
                "AudioTask",   // Name of the task (for debugging)
                8192,          // Stack size (bytes)
                this,          // Parameter to pass
                1,             // Task priority
                &task_handle   // Task handle
            );
        }
        m_f_running = true;
    }
    else
    {
        audiofile.close();
    }
    return ret;
}
#endif // AUDIO_NO_SD_FS
//---------------------------------------------------------------------------------------------------------------------
size_t Audio::readAudioHeader(uint32_t bytes)
{
    size_t bytesReaded = 0;
    if(m_codec == CODEC_WAV)
    {
        if (audiofile.available())
        {
            int res = read_WAV_Header(bytes);
            AUDIO_INFO("read_WAV_Header: res = %d",res);
            if(res >= 0)
            {
                bytesReaded = bytes;
                m_f_running = true;
            }
        }
    }
    if(m_codec == CODEC_RAW)
    {
        // Hardcode those values, at least for now
        setBitsPerSample(16);
        setChannels(1);
        setSampleRate(16000);
        // stream only, no header
        m_f_running = true;
    }
    if(!isRunning())
    {
        log_e("Processing stopped due to invalid audio header");
        return 0;
    }
    return bytesReaded;
}

/* read 4 bytes of data from wav file */
int Audio::read4bytes(File file, uint32_t *chunkId)
{
  int n = file.read((uint8_t *)chunkId, sizeof(uint32_t));
  return n;
}

/* these are function to process wav file */
int Audio::readRiff(File file, wavRiff_t *wavRiff)
{
  int n = file.read((uint8_t *)wavRiff, sizeof(wavRiff_t));
  return n;
}

int Audio::readProps(File file, wavProperties_t *wavProps)
{
  int n = file.read((uint8_t *)wavProps, sizeof(wavProperties_t));
  return n;
}

//---------------------------------------------------------------------------------------------------------------------
int Audio::read_WAV_Header(size_t len)
{
    headerState_t state = HEADER_RIFF;
    wavProperties_t wavProps;
    int n;
    int bytes_read = 0;
    uint32_t chunkId, chunkSize;

    memset(&wavProps,0,sizeof(wavProps));
    while (audiofile.available() && state != DATA)
    {
        switch(state)
        {
        case HEADER_RIFF:
            wavRiff_t wavRiff;
            n = readRiff(audiofile, &wavRiff);
            if(n == sizeof(wavRiff_t))
            {
                if(wavRiff.chunkID == CCCC('R', 'I', 'F', 'F') && wavRiff.format == CCCC('W', 'A', 'V', 'E'))
                {
                    state = HEADER_FMT;
                    bytes_read += n;
                    AUDIO_INFO("HEADER_RIFF found");
                }
            }
            break;
        case HEADER_FMT:
            n = readProps(audiofile, &wavProps);
            if(n == sizeof(wavProperties_t))
            {
                bytes_read += n;
                state = HEADER_DATA;
            }
            else
            {
                AUDIO_INFO("Error reading WAV properties, got %u bytes, should be %u", n, sizeof(wavProperties_t));
                return -1;
            }
            break;
        case HEADER_DATA:
            n = read4bytes(audiofile, &chunkId);
            if(n == 4)
            {
                if(chunkId == CCCC('d', 'a', 't', 'a'))
                {
                    bytes_read += n;
                    AUDIO_INFO("HEADER_DATA");
                }
            }
            n = read4bytes(audiofile, &chunkSize);
            if(n == 4)
            {
                bytes_read += n;
                AUDIO_INFO("WAV DATA");
                state = DATA;
            }
            break; 
        case DATA:
            break;
        }
    }
    if (state == DATA)
    {
        AUDIO_INFO("WAV FormatCode: %u", wavProps.audioFormat);
        AUDIO_INFO("WAV Channels: %u", wavProps.numChannels);
        AUDIO_INFO("WAV DataRate: %u", wavProps.byteRate);
        AUDIO_INFO("WAV DataBlockSize: %u", chunkSize);
        AUDIO_INFO("WAV BitsPerSample: %u", wavProps.bitsPerSample);
        AUDIO_INFO("WAV SampleRate: %u", wavProps.sampleRate);

        if((wavProps.bitsPerSample != 8) && (wavProps.bitsPerSample != 16))
        {
            AUDIO_INFO("BitsPerSample is %u,  must be 8 or 16" , wavProps.bitsPerSample);
            stopSong();
            return -1;
        }
        if((wavProps.numChannels != 1) && (wavProps.numChannels != 2))
        {
            AUDIO_INFO("num channels is %u,  must be 1 or 2" , wavProps.numChannels);
            stopSong();
            return -1;
        }
        if(wavProps.audioFormat != 1)
        {
            AUDIO_INFO("format code is not 1 (PCM) but %u", wavProps.audioFormat);
            stopSong();
            return -1 ; //false;
        }
        if (bytes_read != len)
        {
            AUDIO_INFO("Unexpected header size %u should be %u", bytes_read, len);
            stopSong();
            return -1 ; //false;            
        }
        setBitsPerSample(wavProps.bitsPerSample);
        setChannels(wavProps.numChannels);
        setSampleRate(wavProps.sampleRate);
        return 0;
    }
    return -1; // will never get here for a proper WAV file
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::stopSong() 
{
    uint32_t pos = 0;
    bool wasRunning = m_f_running;
    if(m_f_running) 
    {
        m_f_running = false;
#ifndef AUDIO_NO_SD_FS
        if(getDatamode() == AUDIO_LOCALFILE)
        {
            audiofile.close();
            AUDIO_INFO("Closing audio file");
            Serial.printf("Closed file at %lu\n", millis());
        }
#endif // AUDIO_NO_SD_FS
    }
#ifndef AUDIO_NO_SD_FS
    if(audiofile)
    {
        // added this before putting 'm_f_localfile = false' in stopSong(); shoulf never occur....
        audiofile.close();
        AUDIO_INFO("Closing audio file");
        log_w("Closing audio file");  // for debug
    }
#endif                                               // AUDIO_NO_SD_FS
    memset(m_headerBuff, 0, sizeof(m_headerBuff));
    i2s_zero_dma_buffer((i2s_port_t) m_i2s_num);
    vTaskDelay(1); // allow buffers to be cleared
    // if we were running, check if there's something left in the playlist
    if (wasRunning && getDatamode() == AUDIO_LOCALFILE)
    {
        // Play next file in the playlist (if any)
        if (!m_fileList->empty())
        {
            m_fileListIndex++;
            // reach the end of the playlist?
            if (m_fileListIndex >= m_fileList->size()) 
            {
                m_fileList->clear();
                m_fileListIndex = 0;
            }
            else
            {
                m_fileEndMilis = millis();
                String fileName = m_fileList->at(m_fileListIndex);
                connecttoFS(SD_MMC, fileName);
            }
        }
        else
        {
            m_fileListIndex = 0;
        }
    }
    return pos;
}

//---------------------------------------------------------------------------------------------------------------------
void Audio::loop() 
{
    // no longer used
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio:: initializeDecoder()
{
    switch(m_codec)
    {
        case CODEC_WAV:
            memset(m_headerBuff, 0, sizeof(m_headerBuff));
            readAudioHeader(44); // fixed size for WAV header
            break;
        case CODEC_RAW:
            break;
        default:
            goto exit;
            break;
    }
    return true;

    exit:
        stopSong();
        return false;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::showCodecParams()
{
    AUDIO_INFO("Channels: %i", getChannels());
    AUDIO_INFO("SampleRate: %i", getSampleRate());
    AUDIO_INFO("BitsPerSample: %i", getBitsPerSample());
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setPinout(uint8_t BCLK, uint8_t LRC, uint8_t DOUT, int8_t DIN, int8_t MCK) 
{
    m_pin_config.bck_io_num   = BCLK;
    m_pin_config.ws_io_num    = LRC; //  wclk
    m_pin_config.data_out_num = DOUT;
    m_pin_config.data_in_num  = DIN;
#if(ESP_IDF_VERSION_MAJOR >= 4 && ESP_IDF_VERSION_MINOR >= 4)
    m_pin_config.mck_io_num   = MCK;
#endif
    const esp_err_t result = i2s_set_pin((i2s_port_t) m_i2s_num, &m_pin_config);
    return (result == ESP_OK);
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getFileSize() 
{
#ifdef AUDIO_NO_SD_FS
    return 0;
#else
    if(!audiofile) return 0;
    return audiofile.size();
#endif // AUDIO_NO_SD_FS
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getFilePos() 
{
#ifdef AUDIO_NO_SD_FS
    return 0;
#else
    if(!audiofile) return 0;
    return audiofile.position();
#endif // AUDIO_NO_SD_FS
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setSampleRate(uint32_t sampRate) 
{
    if(!sampRate) sampRate = 16000; // fuse, if there is no value -> set default #209
    i2s_set_sample_rates((i2s_port_t)m_i2s_num, sampRate); // 22050, 32000, 44100, 48000, 96000
    m_sampleRate = sampRate;
#ifndef AUDIO_LIB_LEAN_AND_MEAN
    IIR_calculateCoefficients(m_gain0, m_gain1, m_gain2); // must be recalculated after each samplerate change
#endif
    return true;
}
uint32_t Audio::getSampleRate()
{
    return m_sampleRate;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setBitsPerSample(int bits) 
{
    if((bits != 16) && (bits != 8)) return false;
    m_bitsPerSample = bits;
    return true;
}
uint8_t Audio::getBitsPerSample()
{
    return m_bitsPerSample;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setChannels(int ch) 
{
    if((ch < 1) || (ch > 2)) return false;
    m_channels = ch;
    return true;
}
uint8_t Audio::getChannels()
{
    if (m_channels == 0) {    // this should not happen! #209
        m_channels = 2;
    }
    return m_channels;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::forceMono(bool m) 
{   // #100 mono option
    m_f_forceMono = m; // false stereo, true mono
}
