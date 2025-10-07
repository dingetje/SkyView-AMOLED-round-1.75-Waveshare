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

//---------------------------------------------------------------------------------------------------------------------
AudioBuffer::AudioBuffer(size_t maxBlockSize) 
{
    // if maxBlockSize isn't set use defaultspace (1600 bytes) is enough for aac and mp3 player
    if(maxBlockSize) m_resBuffSizeRAM  = maxBlockSize;
    if(maxBlockSize) m_maxBlockSize = maxBlockSize;
}

AudioBuffer::~AudioBuffer() 
{
    if(m_buffer)
        free(m_buffer);
    m_buffer = NULL;
}

void AudioBuffer::setBufsize(int ram, int psram) 
{
    if (ram > -1) // -1 == default / no change
        m_buffSizeRAM = ram;
    if (psram > -1)
        m_buffSizePSRAM = psram;
}

size_t AudioBuffer::init() 
{
    if(m_buffer) free(m_buffer);
    m_buffer = NULL;
    if(psramInit() && m_buffSizePSRAM > 0) 
    {
        // PSRAM found, AudioBuffer will be allocated in PSRAM
        m_f_psram = true;
        m_buffSize = m_buffSizePSRAM;
        m_buffer = (uint8_t*) ps_calloc(m_buffSize, sizeof(uint8_t));
        m_buffSize = m_buffSizePSRAM - m_resBuffSizePSRAM;
    }
    if(m_buffer == NULL) 
    {
        // PSRAM not found, not configured or not enough available
        m_f_psram = false;
        m_buffSize = m_buffSizeRAM;
        m_buffer = (uint8_t*) calloc(m_buffSize, sizeof(uint8_t));
        m_buffSize = m_buffSizeRAM - m_resBuffSizeRAM;
    }
    if(!m_buffer)
        return 0;
    m_f_init = true;
    resetBuffer();
    return m_buffSize;
}

void AudioBuffer::changeMaxBlockSize(uint16_t mbs)
{
    m_maxBlockSize = mbs;
    return;
}

uint16_t AudioBuffer::getMaxBlockSize()
{
    return m_maxBlockSize;
}

size_t AudioBuffer::freeSpace() 
{
    if(m_readPtr >= m_writePtr) 
    {
        m_freeSpace = (m_readPtr - m_writePtr);
    }
    else 
    {
        m_freeSpace = (m_endPtr - m_writePtr) + (m_readPtr - m_buffer);
    }
    if(m_f_start)
        m_freeSpace = m_buffSize;
    return m_freeSpace - 1;
}

size_t AudioBuffer::writeSpace() 
{
    if(m_readPtr >= m_writePtr) 
    {
        m_writeSpace = (m_readPtr - m_writePtr - 1); // readPtr must not be overtaken
    }
    else 
    {
        if(getReadPos() == 0)
            m_writeSpace = (m_endPtr - m_writePtr - 1);
        else
            m_writeSpace = (m_endPtr - m_writePtr);
    }
    if(m_f_start)
        m_writeSpace = m_buffSize - 1;
    return m_writeSpace;
}

size_t AudioBuffer::bufferFilled() 
{
    if(m_writePtr >= m_readPtr) 
    {
        m_dataLength = (m_writePtr - m_readPtr);
    }
    else
    {
        m_dataLength = (m_endPtr - m_readPtr) + (m_writePtr - m_buffer);
    }
    return m_dataLength;
}

void AudioBuffer::bytesWritten(size_t bw) 
{
    m_writePtr += bw;
    if(m_writePtr == m_endPtr) 
    {
        m_writePtr = m_buffer;
    }
    if(bw && m_f_start)
        m_f_start = false;
}

void AudioBuffer::bytesWasRead(size_t br) 
{
    m_readPtr += br;
    if(m_readPtr >= m_endPtr) 
    {
        size_t tmp = m_readPtr - m_endPtr;
        m_readPtr = m_buffer + tmp;
    }
}

uint8_t* AudioBuffer::getWritePtr() 
{
    return m_writePtr;
}

uint8_t* AudioBuffer::getReadPtr() 
{
    size_t len = m_endPtr - m_readPtr;
    if(len < m_maxBlockSize) // be sure the last frame is completed
    {   
        memcpy(m_endPtr, m_buffer, m_maxBlockSize - len);  // cpy from m_buffer to m_endPtr with len
    }
    return m_readPtr;
}

void AudioBuffer::resetBuffer() 
{
    m_writePtr = m_buffer;
    m_readPtr = m_buffer;
    m_endPtr = m_buffer + m_buffSize;
    m_f_start = true;
    // memset(m_buffer, 0, m_buffSize); //Clear Inputbuffer
    m_complete = false;
}

uint32_t AudioBuffer::getWritePos() 
{
    return m_writePtr - m_buffer;
}

uint32_t AudioBuffer::getReadPos() 
{
    return m_readPtr - m_buffer;
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

#ifndef AUDIO_LIB_LEAN_AND_MEAN
    if(psramInit()) {m_chbufSize = 8192;     m_chbuf = (char*)ps_malloc(m_chbufSize);}
    else            {m_chbufSize = 512 + 64; m_chbuf = (char*)malloc(m_chbufSize);}
#endif

#ifndef AUDIO_LIB_LEAN_AND_MEAN
    clientsecure.setInsecure();  // if that can't be resolved update to ESP32 Arduino version 1.0.5-rc05 or higher
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

    for(int i = 0; i <3; i++) 
    {
        m_filter[i].a0  = 1;
        m_filter[i].a1  = 0;
        m_filter[i].a2  = 0;
        m_filter[i].b1  = 0;
        m_filter[i].b2  = 0;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setBufsize(int rambuf_sz, int psrambuf_sz) 
{
#ifndef AUDIO_LIB_LEAN_AND_MEAN
    if(InBuff.isInitialized()) 
    {
        log_e("Audio::setBufsize must not be called after audio is initialized");
        return;
    }
    InBuff.setBufsize(rambuf_sz, psrambuf_sz);
#endif
};

void Audio::initInBuff() 
{
#ifndef AUDIO_LIB_LEAN_AND_MEAN
    if(!InBuff.isInitialized()) 
    {
        size_t size = InBuff.init();
        if (size > 0) 
        {
            AUDIO_INFO("PSRAM %sfound, inputBufferSize: %u bytes", InBuff.havePSRAM()?"":"not ", size - 1);
        }
    }
    changeMaxBlockSize(1600); // default size mp3 or aac
#endif
}

fs::File Audio::currentFile()
{
    return audiofile;
}

#define I2S_BUFFER_SIZE 512
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
        else if (getDatamode() == AUDIO_LOCALFILE)
        {
            // WAV header already parsed
            File file = currentFile();
            size_t bytes_read, bytes_written;
            // fixed data rate is fine, all audio files are the same
            i2s_set_sample_rates((i2s_port_t)0, 22050);
            while (file.available()) 
            {
                // convert mono data to stereo
                bytes_read = file.read(mono_block, MONO_BLOCK_SIZE);
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
esp_err_t Audio::I2Sstart(uint8_t i2s_num) 
{
    // It is not necessary to call this function after i2s_driver_install() (it is started automatically),
    // however it is necessary to call it after i2s_stop()
    return i2s_start((i2s_port_t) i2s_num);
}

esp_err_t Audio::I2Sstop(uint8_t i2s_num) 
{
    return i2s_stop((i2s_port_t) i2s_num);
}
//---------------------------------------------------------------------------------------------------------------------
esp_err_t Audio::i2s_mclk_pin_select(const uint8_t pin) 
{
    // IDF >= 4.4 use setPinout(BCLK, LRC, DOUT, DIN, MCK) only, i2s_mclk_pin_select() is no longer needed

    if(pin != 0 && pin != 1 && pin != 3) 
    {
        log_e("Only support GPIO0/GPIO1/GPIO3, gpio_num:%d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    #ifdef CONFIG_IDF_TARGET_ESP32
        switch(pin){
            case 0:
                PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
                WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
                break;
            case 1:
                PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
                WRITE_PERI_REG(PIN_CTRL, 0xF0F0);
                break;
            case 3:
                PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
                WRITE_PERI_REG(PIN_CTRL, 0xFF00);
                break;
            default:
                break;
        }
    #endif

    return ESP_OK;
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
    initInBuff(); // initialize InputBuffer if not already done
    InBuff.resetBuffer();    
    AUDIO_INFO("buffers freed, free Heap: %u bytes", ESP.getFreeHeap());

    m_f_chunked = false;                                    // Assume not chunked
    m_f_firstmetabyte = false;
    m_f_playing = false;
    m_f_firstCall = true;                                   // InitSequence for processWebstream and processLokalFile
    m_f_running = false;
    m_streamType = ST_NONE;
    m_codec = CODEC_NONE;
    m_datamode = AUDIO_NONE;
    m_audioFileDuration = 0;
    m_audioDataStart = 0;
    m_audioDataSize = 0;
    m_avr_bitrate = 0;                                      // the same as m_bitrate if CBR, median if VBR
    m_bitRate = 0;                                          // Bitrate still unknown
    m_bytesNotDecoded = 0;                                  // counts all not decodable bytes
    m_chunkcount = 0;                                       // for chunked streams
    m_contentlength = 0;                                    // If Content-Length is known, count it
    m_curSample = 0;
    m_LFcount = 0;                                          // For end of header detection
    m_controlCounter = 0;                                   // Status within readID3data() and readWaveHeader()
    m_channels = 2;                                         // assume stereo #209
    m_streamTitleHash = 0;
    m_file_size = 0;
    m_ID3Size = 0;
}
//---------------------------------------------------------------------------------------------------------------------
#ifndef AUDIO_NO_SD_FS
bool Audio::connecttoSD(const String path, uint32_t resumeFilePos) 
{
    return connecttoFS(SD, path, resumeFilePos);
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
        connecttoFS(SD_MMC, fileName.c_str(), 0);
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
        connecttoFS(SD_MMC, fileName.c_str(), 0);        
    }
}

//---------------------------------------------------------------------------------------------------------------------
bool Audio::connecttoFS(fs::FS &fs, String path, uint32_t resumeFilePos) 
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

    m_resumeFilePos = resumeFilePos;

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
        Serial.printf("Opening file at %lu\n", millis());
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
        int res = read_WAV_Header(InBuff.getReadPtr(), bytes);
        if(res >= 0) bytesReaded = res;
        else
        {   // error, skip header
            m_controlCounter = 100;
        }
    }
    if(m_codec == CODEC_RAW)
    {
        // Hardcode those values, at least for now
        setBitsPerSample(16);
        setChannels(1);
        setSampleRate(16000);
        setBitrate(getChannels() * getSampleRate() * getBitsPerSample());
        // stream only, no header
        m_controlCounter = 100;
    }
    if(!isRunning())
    {
        log_e("Processing stopped due to invalid audio header");
        return 0;
    }
    return bytesReaded;
}
//---------------------------------------------------------------------------------------------------------------------
int Audio::read_WAV_Header(uint8_t* data, size_t len) 
{
    static size_t headerSize;
    static uint32_t cs = 0;
    static uint8_t bts = 0;

    if(m_controlCounter == 0)
    {
        m_controlCounter ++;
        if((*data != 'R') || (*(data + 1) != 'I') || (*(data + 2) != 'F') || (*(data + 3) != 'F')) 
        {
            AUDIO_INFO("file has no RIFF tag");
            headerSize = 0;
            return -1; //false;
        }
        else
        {
            headerSize = 4;
            return 4; // ok
        }
    }

    if(m_controlCounter == 1)
    {
        m_controlCounter ++;
        cs = (uint32_t) (*data + (*(data + 1) << 8) + (*(data + 2) << 16) + (*(data + 3) << 24) - 8);
        headerSize += 4;
        return 4; // ok
    }

    if(m_controlCounter == 2)
    {
        m_controlCounter ++;
        if((*data  != 'W') || (*(data + 1) != 'A') || (*(data + 2) != 'V') || (*(data + 3) != 'E')) 
        {
            AUDIO_INFO("format tag is not WAVE");
            return -1;//false;
        }
        else 
        {
            headerSize += 4;
            return 4;
        }
    }

    if(m_controlCounter == 3)
    {
        if((*data  == 'f') && (*(data + 1) == 'm') && (*(data + 2) == 't')) 
        {
            m_controlCounter ++;
            headerSize += 4;
            return 4;
        }
        else
        {
            headerSize += 4;
            return 4;
        }
    }

    if(m_controlCounter == 4)
    {
        m_controlCounter ++;
        cs = (uint32_t) (*data + (*(data + 1) << 8));
        if(cs > 40) return -1; //false, something going wrong
        bts = cs - 16; // bytes to skip if fmt chunk is >16
        headerSize += 4;
        return 4;
    }

    if(m_controlCounter == 5)
    {
        m_controlCounter ++;
        uint16_t fc  = (uint16_t) (*(data + 0)  + (*(data + 1)  << 8));         // Format code
        uint16_t nic = (uint16_t) (*(data + 2)  + (*(data + 3)  << 8));         // Number of interleaved channels
        uint32_t sr  = (uint32_t) (*(data + 4)  + (*(data + 5)  << 8) +
                                  (*(data + 6)  << 16) + (*(data + 7)  << 24)); // Samplerate
        uint32_t dr  = (uint32_t) (*(data + 8)  + (*(data + 9)  << 8) +
                                  (*(data + 10) << 16) + (*(data + 11) << 24)); // Datarate
        uint16_t dbs = (uint16_t) (*(data + 12) + (*(data + 13) << 8));         // Data block size
        uint16_t bps = (uint16_t) (*(data + 14) + (*(data + 15) << 8));         // Bits per sample

        AUDIO_INFO("FormatCode: %u", fc);
        // AUDIO_INFO("Channel: %u", nic);
        // AUDIO_INFO("SampleRate: %u", sr);
        AUDIO_INFO("DataRate: %u", dr);
        AUDIO_INFO("DataBlockSize: %u", dbs);
        AUDIO_INFO("BitsPerSample: %u", bps);

        if((bps != 8) && (bps != 16))
        {
            AUDIO_INFO("BitsPerSample is %u,  must be 8 or 16" , bps);
            stopSong();
            return -1;
        }
        if((nic != 1) && (nic != 2))
        {
            AUDIO_INFO("num channels is %u,  must be 1 or 2" , nic);
            stopSong();
            return -1;
        }
        if(fc != 1) 
        {
            AUDIO_INFO("format code is not 1 (PCM)");
            stopSong();
            return -1 ; //false;
        }
        setBitsPerSample(bps);
        setChannels(nic);
        setSampleRate(sr);
        setBitrate(nic * sr * bps);
    //    AUDIO_INFO("BitRate: %u", m_bitRate);
        headerSize += 16;
        return 16; // ok
    }

    if(m_controlCounter == 6)
    {
        m_controlCounter ++;
        headerSize += bts;
        return bts; // skip to data
    }

    if(m_controlCounter == 7)
    {
        if((*(data + 0) == 'd') && (*(data + 1) == 'a') && (*(data + 2) == 't') && (*(data + 3) == 'a'))
        {
            m_controlCounter ++;
            vTaskDelay(10);
            headerSize += 4;
            return 4;
        }
        else
        {
            headerSize ++;
            return 1;
        }
    }

    if(m_controlCounter == 8)
    {
        m_controlCounter ++;
        size_t cs =  *(data + 0) + (*(data + 1) << 8) + (*(data + 2) << 16) + (*(data + 3) << 24); //read chunkSize
        headerSize += 4;
        if(getDatamode() == AUDIO_LOCALFILE) m_contentlength = getFileSize();
        if(cs)
        {
            m_audioDataSize = cs  - 44;
        }
        else 
        {   // sometimes there is nothing here
            if(getDatamode() == AUDIO_LOCALFILE) m_audioDataSize = getFileSize() - headerSize;
//            if(m_streamType == ST_WEBFILE) m_audioDataSize = m_contentlength - headerSize;
        }
        AUDIO_INFO("Audio-Length: %u", m_audioDataSize);
        return 4;
    }
    m_controlCounter = 100; // header succesfully read
    m_audioDataStart = headerSize;
    return 0;
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
            m_streamType = ST_NONE;
            pos = getFilePos() - inBufferFilled();
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
#endif                                           // AUDIO_NO_SD_FS
    memset(m_outBuff, 0, sizeof(m_outBuff));     //Clear OutputBuffer
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
            InBuff.changeMaxBlockSize(m_frameSizeWav);
            break;
        case CODEC_RAW:
            InBuff.changeMaxBlockSize(m_frameSizeRaw);
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
    // print Codec Parameter (mp3, aac) in audio_info()

    AUDIO_INFO("Channels: %i", getChannels());
    AUDIO_INFO("SampleRate: %i", getSampleRate());
    AUDIO_INFO("BitsPerSample: %i", getBitsPerSample());
    if(getBitRate()) {AUDIO_INFO("BitRate: %i", getBitRate());}
    else             {AUDIO_INFO("BitRate: N/A");}
}
//---------------------------------------------------------------------------------------------------------------------
int Audio::findNextSync(uint8_t* data, size_t len)
{
    // Mp3 and aac audio data are divided into frames. At the beginning of each frame there is a sync word.
    // The sync word is 0xFFF. This is followed by information about the structure of the frame.
    // Wav files have no frames
    // Return: 0 the synchronous word was found at position 0
    //         > 0 is the offset to the next sync word
    //         -1 the sync word was not found within the block with the length len

    int nextSync = 0;
    if(m_codec == CODEC_WAV)  
    {
        m_f_playing = true; nextSync = 0;
    }
    if(m_codec == CODEC_RAW) 
    {
        m_f_playing = true; nextSync = 0;
    }
     return nextSync;
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
uint32_t Audio::getAudioDataStartPos() 
{
#ifdef AUDIO_NO_SD_FS
    return 0;
#else
    if(!audiofile) return 0;
    return m_audioDataStart;
#endif // AUDIO_NO_SD_FS
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getAudioFileDuration() 
{
#ifndef AUDIO_NO_SD_FS
    if(getDatamode() == AUDIO_LOCALFILE) {if(!audiofile) return 0;}
#endif
     if(m_avr_bitrate && m_codec == CODEC_WAV)   m_audioFileDuration = 8 * (m_audioDataSize / m_avr_bitrate);
    else if(m_avr_bitrate && m_codec == CODEC_RAW)   m_audioFileDuration = 8 * (m_audioDataSize / m_avr_bitrate);
    else return 0;
    return m_audioFileDuration;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setAudioPlayPosition(uint16_t sec)
{
    // Jump to an absolute position in time within an audio file
    // e.g. setAudioPlayPosition(300) sets the pointer at pos 5 min
    // works only with format mp3 or wav
    if(m_codec == CODEC_M4A)  return false;
    if(sec > getAudioFileDuration()) sec = getAudioFileDuration();
    uint32_t filepos = m_audioDataStart + (m_avr_bitrate * sec / 8);

    return setFilePos(filepos);
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setTimeOffset(int sec)
{
    // fast forward or rewind the current position in seconds
    // audiosource must be a mp3, aac or wav file

#ifndef AUDIO_NO_SD_FS

    if(!audiofile || !m_avr_bitrate) return false;

    uint32_t oneSec  = m_avr_bitrate / 8;                   // bytes decoded in one sec
    int32_t  offset  = oneSec * sec;                        // bytes to be wind/rewind
    uint32_t startAB = m_audioDataStart;                    // audioblock begin
    uint32_t endAB   = m_audioDataStart + m_audioDataSize;  // audioblock end

    if(m_codec == CODEC_MP3 || m_codec == CODEC_AAC || m_codec == CODEC_WAV || m_codec == CODEC_RAW || m_codec == CODEC_FLAC)
    {
        int32_t pos = getFilePos();
        pos += offset;
        if(pos <  (int32_t)startAB) pos = startAB;
        if(pos >= (int32_t)endAB)   pos = endAB;
        setFilePos(pos);
        return true;
    }
#endif // AUDIO_NO_SD_FS
    return false;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setFilePos(uint32_t pos) 
{
#ifdef AUDIO_NO_SD_FS
    return false;
#else
    if(!audiofile) return false;
    if(pos < m_audioDataStart) pos = m_audioDataStart; // issue #96
    if(pos > m_file_size) pos = m_file_size;
    m_resumeFilePos = pos;
    return true;
#endif // AUDIO_NO_SD_FS
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::audioFileSeek(const float speed) {
    // 0.5 is half speed
    // 1.0 is normal speed
    // 1.5 is one and half speed
    if((speed > 1.5f) || (speed < 0.25f)) return false;

    uint32_t srate = getSampleRate() * speed;
    i2s_set_sample_rates((i2s_port_t)m_i2s_num, srate);
    return true;
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
bool Audio::setBitrate(int br)
{
    m_bitRate = br;
    if(br)return true;
    return false;
}
uint32_t Audio::getBitRate(bool avg)
{
    if (avg)
        return m_avr_bitrate;
    return m_bitRate;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::forceMono(bool m) 
{   // #100 mono option
    m_f_forceMono = m; // false stereo, true mono
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setBalance(int8_t bal)
{   // bal -16...16
    if(bal < -16) bal = -16;
    if(bal >  16) bal =  16;
    m_balance = bal;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setVolume(uint8_t vol) 
{   // vol 22 steps, 0...21
    if(vol > 21) vol = 21;
    m_vol = volumetable[vol];
}
//---------------------------------------------------------------------------------------------------------------------
uint8_t Audio::getVolume() 
{
    for(uint8_t i = 0; i < 22; i++) {
        if(volumetable[i] == m_vol) return i;
    }
    m_vol = 12; // if m_vol not found in table
    return m_vol;
}
//---------------------------------------------------------------------------------------------------------------------
uint8_t Audio::getI2sPort() 
{
    return m_i2s_num;
}
//---------------------------------------------------------------------------------------------------------------------
int32_t Audio::Gain(int16_t s[2]) 
{
    int32_t v[2];
    float step = (float)m_vol /64;
    uint8_t l = 0, r = 0;

    if(m_balance < 0)
    {
        step = step * (float)(abs(m_balance) * 4);
        l = (uint8_t)(step);
    }
    if(m_balance > 0)
    {
        step = step * m_balance * 4;
        r = (uint8_t)(step);
    }

    v[LEFTCHANNEL] = (s[LEFTCHANNEL]  * (m_vol - l)) >> 6;
    v[RIGHTCHANNEL]= (s[RIGHTCHANNEL] * (m_vol - r)) >> 6;

    return (v[LEFTCHANNEL] << 16) | (v[RIGHTCHANNEL] & 0xffff);
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::inBufferFilled() 
{
    // current audio input buffer fillsize in bytes
    return InBuff.bufferFilled();
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::inBufferFree() 
{
    // current audio input buffer free space in bytes
    return InBuff.freeSpace();
}
