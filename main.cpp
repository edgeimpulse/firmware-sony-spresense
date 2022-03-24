#include <stdio.h>
#include <sys/boardctl.h>
#include <time.h>
#include <arch/board/board.h>
#include <arch/cxd56xx/pin.h>
#include <cxd56_uart.h>
#include <hardware/cxd5602_memorymap.h>

#include "Audio.h"
#include "Wire.h"
#include "KX126.h"
#include "File.h"

#include "ei_device_sony_spresense_config.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* Extern reference -------------------------------------------------------- */
extern int ei_main();
extern void set_id(char* id);

/* Forward declarations ---------------------------------------------------- */
static void disable_uart_irq(void);
static void handle_sony_id(void);
static void init_acc(void);
static void audio_attention_cb(const ErrorAttentionParam *atprm);
static void empty_audio_fifo(void);

/* Registers read/write macro's ------------------------------------------- */
#define putreg32(v,a)   (*(volatile uint32_t *)(a) = (v))
#define getreg32(a)     (*(volatile uint32_t *)(a))
#define CONSOLE_BASE    CXD56_UART1_BASE

/* Private variables ------------------------------------------------------- */
KX126 kx126(KX126_DEVICE_ADDRESS_1F);

/* Audio variables */
AudioClass *theAudio;
static const int32_t buffer_size = 1600; /*768sample,1ch,16bit*/

extern "C" {

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (*func)()) __attribute__((weak));
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
typedef void (*initializer_t)(void);
extern initializer_t _sinit;
extern initializer_t _einit;
extern uint32_t _stext;
extern uint32_t _etext;

static void up_cxxinitialize(void)
{
    initializer_t *initp;

    /* Visit each entry in the initialization table */

    for (initp = &_sinit; initp != &_einit; initp++) {
        initializer_t initializer = *initp;

        /* Make sure that the address is non-NULL and lies in the text region
         * defined by the linker script.  Some toolchains may put NULL values
         * or counts in the initialization table.
         */

        if ((void *)initializer > (void *)&_stext && (void *)initializer < (void *)&_etext) {
            initializer();
        }
    }
}
#endif
}

File SdFile = NULL; /**< File object */

/**
 * @brief Main application function
 *
 */
extern "C" int spresense_main(void)
{
#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
    up_cxxinitialize();
#endif
    boardctl(BOARDIOC_INIT, 0);

    disable_uart_irq();
    handle_sony_id();

    init_acc();

    ei_main();
    return 0;
}

/**
 * @brief Get current time from spresense lib
 *
 * @param sec
 * @param nano
 */
extern "C" void spresense_time_cb(uint32_t *sec, uint32_t *nano)
{
    struct timespec cur_time;
    clock_gettime(CLOCK_MONOTONIC, &cur_time);
    *(sec) = cur_time.tv_sec;
    *(nano)= cur_time.tv_nsec;
}

/**
 * @brief Get a char directly from the UART
 *
 * @return char
 */
char spresense_getchar(void)
{
    uint32_t reg = 0;
    reg = getreg32(CONSOLE_BASE + CXD56_UART_FR);
    if ((UART_FR_RXFE & reg) && !(UART_FR_RXFF & reg)) {
        return 0;
    }
    else {
        return (char)getreg32(CONSOLE_BASE + CXD56_UART_DR);
    }
}

/**
 * @brief Write a byte straight to the UART DR register
 *
 * @param byte
 */
void spresense_putchar(char byte)
{
    while ((getreg32(CONSOLE_BASE + CXD56_UART_FR) & UART_FLAG_TXFF));
    putreg32((uint32_t)byte, CONSOLE_BASE + CXD56_UART_DR);
}

/**
 * @brief Control board leds using lib functions
 *
 * @param led
 * @param on_off
 */
extern "C" void spresense_ledcontrol(tEiLeds led, bool on_off)
{
    switch (led)
    {
        case LED1: board_gpio_write(PIN_I2S1_DATA_OUT, on_off); break;
        case LED2: board_gpio_write(PIN_I2S1_DATA_IN, on_off);  break;
        case LED3: board_gpio_write(PIN_I2S1_LRCK, on_off);     break;
        case LED4: board_gpio_write(PIN_I2S1_BCK, on_off);      break;
        default: //all leds
        {
            board_gpio_write(PIN_I2S1_DATA_OUT, on_off);
            board_gpio_write(PIN_I2S1_DATA_IN, on_off);
            board_gpio_write(PIN_I2S1_LRCK, on_off);
            board_gpio_write(PIN_I2S1_BCK, on_off);
            break;
        }
    }
}

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
    * Set PCM capture sapling rate parameters to 48 kb/s. Set channel number 4
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

    if(start == true) {
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
    if(pause == true) {
        theAudio->stopRecorder();
    }
    else {
        theAudio->startRecorder();
    }
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

    if(((err == AUDIOLIB_ECODE_OK) || (err == AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA)) && (*size > 0)) {
        data_ready = true;
    }

    return data_ready;
}

/**
 * @brief Open a file from the SD Card
 *
 * @param name
 * @param write if true data can be written to the file
 * @return true success
 */
extern "C" bool spresense_openFile(const char *name, bool write)
{
    bool success;

    /* If file already open, try to close it */
    if(SdFile) {
        SdFile.close();
    }

    SdFile = write == true ? File(name, FILE_WRITE) : File(name);

    /* Check if opened & set to start position */
    if (SdFile) {
        SdFile.seek(0);
        success = true;
    }
    else {
        success = false;
    }

    return success;
}

/**
 * @brief Close the file
 *
 * @param name
 * @return true success
 */
extern "C" bool spresense_closeFile(const char *name)
{
    bool success;

    if(SdFile) {

        /* Close the file */
        SdFile.close();
        SdFile = NULL;
        success = true;
    }
    else {
        ei_printf("File %s not open\r\n", name);
        success = false;
    }

    return success;
}

/**
 * @brief Write data to an opened file
 *
 * @param name
 * @param buf
 * @param length in bytes
 * @return true success
 */
extern "C" bool spresense_writeToFile(const char *name, const uint8_t *buf, uint32_t length)
{
    bool success;

    /* Check if file is opened */
    if (SdFile) {

        SdFile.write((const uint8_t *)buf, length);
        success = true;
    }
    else {
        ei_printf("File %s not open\r\n", name);
        success = false;
    }

    return success;
}

/**
 * @brief Read data from an opened file
 *
 * @param name
 * @param buf
 * @param length in bytes
 * @return uint32_t 0 if ok
 */
extern "C" uint32_t spresense_readFromFile(const char *name, uint8_t *buf, uint32_t length)
{
    uint32_t retVal = 0;

    if (SdFile) {

        if(length > SdFile.available()) {
            ei_printf("File invalid length error\r\n");
            retVal = -1;
        }

        if(SdFile.read((void *)buf, length) < 0){
            ei_printf("File read error\r\n");
            retVal = -2;
        }
    }
    else {
        ei_printf("File %s not open\r\n", name);
        retVal = -3;
    }

    return retVal;
}

/* Private functions ------------------------------------------------------- */

/**
 * @brief IRQ is attached to NUTTX handler. Disable to you use Uart directly.
 */
void disable_uart_irq(void)
{
    putreg32((uint32_t)0x0, CONSOLE_BASE + CXD56_UART_IMSC);
}

/**
 * @brief Get device ID and write to device settings
 *
 */
static void handle_sony_id(void)
{
    uint8_t raw_id[16] = {0};
    char id_string[32];

    boardctl(BOARDIOC_UNIQUEID, (uintptr_t)raw_id);

    sprintf(id_string, "%X:%X:%X:%X:%X", raw_id[0], raw_id[1], raw_id[2], raw_id[3], raw_id[4]);

    set_id(id_string);
}

/**
 * @brief Init the accelerometer
 * @todo  Should we support the KX122?
 */
static void init_acc(void)
{
    Wire.begin();

    uint8_t rc = kx126.init();

    if (rc != 0) {
        ei_printf("Accelerometer (KX126) missing or not working correctly\r\n");
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

    if(!s_buffer) {
      ei_printf("Memory allocation failed\r\n");
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

/**
 * @brief Call cxd56 driver to set baudrate to the max speed
 */
void set_max_data_output_baudrate_c()
{
    cxd56_setbaud(CONSOLE_BASE, BOARD_UART1_BASEFREQ, MAX_BAUD);
}

/**
 * @brief Call cxd56 driver to set baudrate to the default speed
 */
void set_default_data_output_baudrate_c()
{
    cxd56_setbaud(CONSOLE_BASE, BOARD_UART1_BASEFREQ, DEFAULT_BAUD);
}
