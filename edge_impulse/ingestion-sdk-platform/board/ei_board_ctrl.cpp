
#include "ei_board_ctrl.h"
#include "KX126.h"
#include "Wire.h"
#include "Audio.h"

#include "File.h"

/* Private variables ------------------------------------------------------- */
KX126 kx126(KX126_DEVICE_ADDRESS_1F);

/* Audio variables */
static AudioClass *theAudio;
static const int32_t buffer_size = 1600; /*768sample,1ch,16bit*/

static File SdFile = NULL; /**< File object */

/* Private functions ------------------------------------------------------- */
static void empty_audio_fifo(void);

/**
 * @brief Init the accelerometer
 * @todo  Should we support the KX122?
 */
bool spresense_init_acc(void)
{
    Wire.begin();

    uint8_t rc = kx126.init();
    
    if (rc != 0) {
        Wire.end();
        return false;        
    }
    
    return true;
}

/**
 * @brief 
 * 
 * @param acc_val 
 * @return int 
 */
int spresense_getAcc(float acc_val[3])
{
    return (int)kx126.get_val(acc_val);
}

/**
 * @brief Create audio instance and setup audio channel
 * @details Uses PCM format MONO @ 16KHz
 *
 * @return int
 */
int spresense_setupAudio(void)
{
    int retVal = 0;

    theAudio = AudioClass::getInstance();

    retVal = theAudio->begin();

    /* Select input device as microphone */
    retVal |= theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC, 220, SIMPLE_FIFO_BUF_SIZE, false);

    /*
    * Set PCM capture sapling rate parameters to 16 kb/s. Set channel mono
    * Search for PCM codec in "/mnt/sd0/BIN" directory
    */
    retVal |= theAudio->initRecorder(AS_CODECTYPE_PCM, "/mnt/sd0/BIN", AS_SAMPLINGRATE_16000, AS_CHANNEL_MONO);

    return retVal;
}

/**
 * @brief Send command to start recording audio
 *
 */
bool spresense_startStopAudio(bool start)
{
    bool cmdOk = true;

    if (start == true) {
        cmdOk = spresense_setupAudio() ? false : true;
        theAudio->startRecorder();
    }
    else {
        theAudio->stopRecorder();
        empty_audio_fifo();
        theAudio->setReadyMode();
        theAudio->end();
    }

    return cmdOk;
}

/**
 * @brief Stop audio recording without releasing memory
 *
 * @param pause
 */
void spresense_pauseAudio(bool pause)
{
    if (pause == true) {
        theAudio->stopRecorder();
    }
    else {
        theAudio->startRecorder();
    }
}

/**
 * @brief 
 * 
 */
void spresense_closeAudio(void)
{
    empty_audio_fifo();
    theAudio->setReadyMode();
    theAudio->end();
}

/**
 * @brief Puts a sample array in audio_buffer with size in bytes
 *
 * @param audio_buffer
 * @param size
 * @return true when audio_buffer is filled
 */
bool spresense_getAudio(char *audio_buffer, unsigned int* size)
{
    bool data_ready = false;

    err_t err = theAudio->readFrames(audio_buffer, buffer_size, (uint32_t *)size);

    if (((err == AUDIOLIB_ECODE_OK) || (err == AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA)) && (*size > 0)) {
        data_ready = true;
    }

    return data_ready;
}

/**
 * @brief 
 * 
 * @param name 
 * @param buf 
 * @param length 
 * @return uint32_t 
 */
uint32_t spresense_readFromFile(const char *name, uint8_t *buf, uint32_t pos, uint32_t length)
{
    uint32_t retVal = 0;

    if (SdFile) {
        if (SdFile.available() > 0) {
            #if 0
            if (length > SdFile.available()) {            
                length = SdFile.available();
            }
            
            if (SdFile.seek(pos) == false) {
                return -4;
            }
            #endif
            if(length > SdFile.available()) {
                //ei_printf("File invalid length error\r\n");
                retVal = -1;
            }

            if (SdFile.read((void *)buf, length) < 0) {            
                retVal = -2;
            }
            else {
                retVal = length;
            }
        }

    }
    else {        
        retVal = -3;
    }

    return retVal;
}

/**
 * @brief Write data to an opened file
 *
 * @param name
 * @param buf
 * @param length in bytes
 * @return true success
 */
int spresense_writeToFile(const char *name, const uint8_t *buf, uint32_t length)
{
    int success = 0;

    /* Check if file is opened */
    if (SdFile) {
        success = SdFile.write((const uint8_t *)buf, length);    
    }
    else {
        success = 0;
    }

    return success;
}

/**
 * @brief Close the file
 *
 * @param name
 * @return true success
 */
int spresense_closeFile(const char *name)
{
    int success = 0;

    if (SdFile) {
        /* Close the file */
        SdFile.close();
        SdFile = NULL;
    }
    else {
        success = 1;
    }

    return success;
}

/**
 * @brief Open a file from the SD Card
 *
 * @param name
 * @param write if true data can be written to the file
 * @return true success
 */
int spresense_openFile(const char *name, bool write)
{
    int success = 0;

    /* If file already open, try to close it */
    if (SdFile) {
        SdFile.close();
    }

    SdFile = write == true ? File(name, FILE_WRITE) : File(name);

    /* Check if opened & set to start position */
    if (!SdFile) {        
        success = 1;
    }

    return success;
}

/**
 * @brief 
 * 
 * @param name 
 * @return int 
 */
int spresense_deleteFile(const char *name)
{
    int success = 0;

    if (!SdFile) {
        return -1;
    }
    SdFile = File(name, FILE_DELETE);

    if (!SdFile) {
        success = 2;
    }

    return success;
}

/**
 * @brief 
 * 
 */
void spresense_rewind(void)
{
    if (SdFile) {
        SdFile.seek(0);
    }    
}

/**
 * @brief Execute frames for FIFO empty
 */
static void empty_audio_fifo(void)
{
    uint32_t read_size = 0;

    char *s_buffer;

    s_buffer = (char *)malloc(buffer_size);

    if (!s_buffer) {
      //ei_printf("Memory allocation failed\r\n");
    }

    do
    {
        err_t err = theAudio->readFrames(s_buffer, buffer_size, &read_size);
        if ((err != AUDIOLIB_ECODE_OK)
        && (err != AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA)) {
            break;
        }
    }
    while (read_size > 0);
    free(s_buffer);
}
