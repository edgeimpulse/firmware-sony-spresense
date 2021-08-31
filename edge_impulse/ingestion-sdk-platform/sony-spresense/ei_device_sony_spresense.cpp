/* Edge Impulse ingestion SDK
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include "ei_device_sony_spresense.h"
#include "ei_sony_spresense_fs_commands.h"
#include "ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/ei_utils.h"
#include "ei_inertialsensor.h"
#include "ei_microphone.h"
#include "repl.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>


/** Max size for device id array */
#define DEVICE_ID_MAX_SIZE 32

/** Sensors */
typedef enum
{
    MICROPHONE = 0,
    ACCELEROMETER
} used_sensors_t;


/** Data Output Baudrate */
const ei_device_data_output_baudrate_t ei_dev_max_data_output_baudrate = {
    xstr(MAX_BAUD),
    MAX_BAUD,
};

const ei_device_data_output_baudrate_t ei_dev_default_data_output_baudrate = {
    xstr(DEFAULT_BAUD),
    DEFAULT_BAUD,
};

// forward declare the sony uart driver func b/c the header doesn't play nice
extern "C" void cxd56_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud);
#define BOARD_FCLKOUT_FREQUENCY   (100000000)
/* UART clocking ***********************************************************/
#define BOARD_UART1_BASEFREQ        BOARD_FCLKOUT_FREQUENCY

/** Device type */
static const char *ei_device_type = "SONY_SPRESENSE";

/** Device id array */
static char ei_device_id[DEVICE_ID_MAX_SIZE];

/** Device object, for this class only 1 object should exist */
EiDeviceSonySpresense EiDevice;

static tEiState ei_program_state = eiStateIdle;


/* Private function declarations ------------------------------------------- */
static int get_id_c(uint8_t out_buffer[32], size_t *out_size);
static int set_id_c(char *device_id);
static int get_type_c(uint8_t out_buffer[32], size_t *out_size);
static bool get_wifi_connection_status_c(void);
static bool get_wifi_present_status_c(void);
// static void timer_callback(void *arg);
static bool read_sample_buffer(size_t begin, size_t length, void (*data_fn)(uint8_t *, size_t));
static int get_data_output_baudrate_c(ei_device_data_output_baudrate_t *baudrate);

extern char spresense_getchar(void);
extern void spresense_putchar(char byte);
extern "C" void spresense_ledcontrol(uint32_t led, bool on_off);

/* Public functions -------------------------------------------------------- */

EiDeviceInfo* EiDeviceInfo::get_device() { return &EiDevice; }

EiDeviceSonySpresense::EiDeviceSonySpresense(void)
{
}

/**
 * @brief      For the device ID, the BLE mac address is used.
 *             The mac address string is copied to the out_buffer.
 *
 * @param      out_buffer  Destination array for id string
 * @param      out_size    Length of id string
 *
 * @return     0
 */
int EiDeviceSonySpresense::get_id(uint8_t out_buffer[32], size_t *out_size)
{
    return get_id_c(out_buffer, out_size);
}

/**
 * @brief      Gets the identifier pointer.
 *
 * @return     The identifier pointer.
 */
const char *EiDeviceSonySpresense::get_id_pointer(void)
{
    return (const char *)ei_device_id;
}

/**
 * @brief      Copy device type in out_buffer & update out_size
 *
 * @param      out_buffer  Destination array for device type string
 * @param      out_size    Length of string
 *
 * @return     -1 if device type string exceeds out_buffer
 */
int EiDeviceSonySpresense::get_type(uint8_t out_buffer[32], size_t *out_size)
{
    return get_type_c(out_buffer, out_size);
}

/**
 * @brief      Gets the type pointer.
 *
 * @return     The type pointer.
 */
const char *EiDeviceSonySpresense::get_type_pointer(void)
{
    return (const char *)ei_device_type;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceSonySpresense::get_wifi_connection_status(void)
{
    return false;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceSonySpresense::get_wifi_present_status(void)
{
    return false;
}

/**
 * @brief      Create sensor list with sensor specs
 *             The studio and daemon require this list
 * @param      sensor_list       Place pointer to sensor list
 * @param      sensor_list_size  Write number of sensors here
 *
 * @return     False if all went ok
 */
bool EiDeviceSonySpresense::get_sensor_list(
    const ei_device_sensor_t **sensor_list,
    size_t *sensor_list_size)
{
    /* Calculate number of bytes available on flash for sampling, reserve 1 block for header + overhead */
    uint32_t available_bytes = (ei_sony_spresense_fs_get_n_available_sample_blocks() - 1) *
        ei_sony_spresense_fs_get_block_size();

    sensors[MICROPHONE].name = "Built-in microphone";
    sensors[MICROPHONE].start_sampling_cb = &ei_microphone_sample_start;
    sensors[MICROPHONE].max_sample_length_s = available_bytes / (16000 * 2);
    sensors[MICROPHONE].frequencies[0] = 16000.0f;


    sensors[ACCELEROMETER].name = "Built-in accelerometer";
    sensors[ACCELEROMETER].start_sampling_cb = &ei_inertial_setup_data_sampling;

    sensors[ACCELEROMETER].max_sample_length_s = available_bytes / (100 * SIZEOF_N_AXIS_SAMPLED);
    sensors[ACCELEROMETER].frequencies[0] = 62.5f;
    sensors[ACCELEROMETER].frequencies[1] = 250.0f;
    sensors[ACCELEROMETER].frequencies[2] = 500.0f;


    *sensor_list = sensors;
    *sensor_list_size = EI_DEVICE_N_SENSORS;

    return false;
}

/**
 * @brief      Create resolution list for snapshot setting
 *             The studio and daemon require this list
 * @param      snapshot_list       Place pointer to resolution list
 * @param      snapshot_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */
bool EiDeviceSonySpresense::get_snapshot_list(
    const ei_device_snapshot_resolutions_t **snapshot_list,
    size_t *snapshot_list_size,
    const char **color_depth)
{
    snapshot_resolutions[0].width = 320;
    snapshot_resolutions[0].height = 240;
    snapshot_resolutions[1].width = 160;
    snapshot_resolutions[1].height = 160;
    snapshot_resolutions[2].width = 96;
    snapshot_resolutions[2].height = 96;
    snapshot_resolutions[3].width = 96;
    snapshot_resolutions[3].height = 64;

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA
    snapshot_resolutions[4].width = EI_CLASSIFIER_INPUT_WIDTH;
    snapshot_resolutions[4].height = EI_CLASSIFIER_INPUT_HEIGHT;
#endif

    *snapshot_list = snapshot_resolutions;
    *snapshot_list_size = EI_DEVICE_N_RESOLUTIONS;
    *color_depth = "RGB";

    return false;
}

/**
 * @brief      Device specific delay ms implementation
 *
 * @param[in]  milliseconds  The milliseconds
 */
void EiDeviceSonySpresense::delay_ms(uint32_t milliseconds)
{
    ei_sleep(milliseconds);
}

void EiDeviceSonySpresense::setup_led_control(void)
{
    // tHalTmr *AppTmr = NULL;
    // /* Create Periodic timer */
    // AppTmr = HalTmrCreate(HalTmrCh0, HalTmrPeriodic, 1000, timer_callback, AppTmr);
    // if (!AppTmr)
    //     ei_printf("TImer Creatation Failed\n");
    // else
    //     HalTmrStart(AppTmr);
}

void EiDeviceSonySpresense::set_state(tEiState state)
{
    ei_program_state = state;

    if (state == eiStateFinished) {
        spresense_ledcontrol(LEDALL, 0);
        EiDevice.delay_ms(100);
        spresense_ledcontrol(LED1, 1);
        EiDevice.delay_ms(100);
        spresense_ledcontrol(LED2, 1);
        EiDevice.delay_ms(100);
        spresense_ledcontrol(LED3, 1);
        EiDevice.delay_ms(100);
        spresense_ledcontrol(LED4, 1);
        EiDevice.delay_ms(200);
        spresense_ledcontrol(LEDALL, 0);
        EiDevice.delay_ms(200);
        spresense_ledcontrol(LEDALL, 1);
        EiDevice.delay_ms(200);
        spresense_ledcontrol(LEDALL, 0);

        ei_program_state = eiStateIdle;
    }
}

/**
 * @brief      Get the data output baudrate
 *
 * @param      baudrate    Baudrate used to output data
 *
 * @return     0
 */
int EiDeviceSonySpresense::get_data_output_baudrate(ei_device_data_output_baudrate_t *baudrate)
{
    return get_data_output_baudrate_c(baudrate);
}

/**
 * @brief      Set output baudrate to max
 *
 */
void EiDeviceSonySpresense::set_max_data_output_baudrate()
{
    set_max_data_output_baudrate_c();
}

/**
 * @brief      Set output baudrate to default
 *
 */
void EiDeviceSonySpresense::set_default_data_output_baudrate()
{
    set_default_data_output_baudrate_c();
}

/**
 * @brief      Get a C callback for the get_id method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceSonySpresense::get_id_function(void)
{
    return &get_id_c;
}

/**
 * @brief      Get a C callback for the set_id method
 *
 * @return     Pointer to c get function
 */
c_callback_set_id EiDeviceSonySpresense::set_id_function(void)
{
    return &set_id_c;
}

/**
 * @brief      Set the device ID
 *
 * @param      device_id   MAC address
 *
 * @return     0
 */
int EiDeviceSonySpresense::set_id(char *device_id)
{
    return set_id_c(device_id);
}

/**
 * @brief      Get a C callback for the get_type method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceSonySpresense::get_type_function(void)
{
    return &get_type_c;
}

/**
 * @brief      Get a C callback for the get_wifi_connection_status method
 *
 * @return     Pointer to c get function
 */
c_callback_status EiDeviceSonySpresense::get_wifi_connection_status_function(void)
{
    return &get_wifi_connection_status_c;
}

/**
 * @brief      Get a C callback for the wifi present method
 *
 * @return     The wifi present status function.
 */
c_callback_status EiDeviceSonySpresense::get_wifi_present_status_function(void)
{
    return &get_wifi_present_status_c;
}

/**
 * @brief      Get a C callback to the read sample buffer function
 *
 * @return     The read sample buffer function.
 */
c_callback_read_sample_buffer EiDeviceSonySpresense::get_read_sample_buffer_function(void)
{
    return &read_sample_buffer;
}

/**
 * @brief      Get characters for uart pheripheral and send to repl
 */
void ei_command_line_handle(void)
{
    uint8_t data;

    data = spresense_getchar();

    if(data != 0) {
        rx_callback(data);
    }
}

/**
 * @brief      Setup the serial port
 */
void ei_serial_setup(void)
{

}

/**
 * @brief      Printf function uses vsnprintf and output to UART
 *
 * @param[in]  format     Variable argument list
 */
void ei_printf(const char *format, ...)
{
    char buffer[256];
    int length;
    va_list myargs;
    va_start(myargs, format);
    length = vsprintf(buffer, format, myargs);
    va_end(myargs);

    for(int i = 0; i < length; i++) {
        ei_putc(buffer[i]);
    }
}

/**
 * @brief      Print a float value, bypassing the stdio %f
 *             Uses standard serial out
 *
 * @param[in]  f     Float value to print.
 */
void ei_printf_float(float f)
{
    ei_printf("%f", f);
}

/**
 * @brief      Write serial data with length to Serial output
 *
 * @param      data    The data
 * @param[in]  length  The length
 */
void ei_write_string(char *data, int length)
{
    for (int i = 0; i < length; i++) {
        ei_putc(*(data++));
    }
}

/**
 * @brief      Write single character to serial output
 *
 * @param[in]  cChar  The character
 */
void ei_putc(char cChar)
{
    spresense_putchar(cChar);
}

/* Private functions ------------------------------------------------------- */
static void timer_callback(void *arg)
{
    static char toggle = 0;

    if (toggle) {
        switch (ei_program_state) {
        case eiStateErasingFlash:
            spresense_ledcontrol(LED1, 1);
            break;
        case eiStateSampling:
            spresense_ledcontrol(LED2, 1);
            break;
        case eiStateUploading:
            spresense_ledcontrol(LED3, 1);
            break;
        default:
            break;
        }
    }
    else {
        if (ei_program_state != eiStateFinished) {
            spresense_ledcontrol(LEDALL, 0);
        }
    }
    toggle ^= 1;
}


char ei_getchar()
{
    return spresense_getchar();

}


static int get_id_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_id);

    if (length < 32) {
        memcpy(out_buffer, ei_device_id, length);

        *out_size = length;
        return 0;
    }

    else {
        *out_size = 0;
        return -1;
    }
}

static int set_id_c(char *device_id)
{
    size_t length = strlen(ei_device_id);
    if (length > 31) {
        return -1;
    }

    memcpy(ei_device_id, device_id, strlen(device_id) + 1);

    return 0;
}

static int get_type_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_type);

    if (length < 32) {
        memcpy(out_buffer, ei_device_type, length);

        *out_size = length;
        return 0;
    }

    else {
        *out_size = 0;
        return -1;
    }
}

static bool get_wifi_connection_status_c(void)
{
    return false;
}

static bool get_wifi_present_status_c(void)
{
    return false;
}

static int get_data_output_baudrate_c(ei_device_data_output_baudrate_t *baudrate)
{
    size_t length = strlen(ei_dev_max_data_output_baudrate.str);

    if (length < 32) {
        memcpy(baudrate, &ei_dev_max_data_output_baudrate, sizeof(ei_device_data_output_baudrate_t));
        return 0;
    }
    else {
        return -1;
    }
}

/**
 * @brief      Read samples from sample memory and send to data_fn function
 *
 * @param[in]  begin    Start address
 * @param[in]  length   Length of samples in bytes
 * @param[in]  data_fn  Callback function for sample data
 *
 * @return     false on flash read function
 */
static bool read_sample_buffer(size_t begin, size_t length, void (*data_fn)(uint8_t *, size_t))
{
    size_t pos = begin;
    size_t bytes_left = length;
    bool retVal;

    EiDevice.set_state(eiStateUploading);

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];
    while (1) {
        size_t bytes_to_read = sizeof(buffer);
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            retVal = true;
            break;
        }

        int r = ei_sony_spresense_fs_read_sample_data(buffer, pos, bytes_to_read);
        if (r != 0) {
            retVal = false;
            break;
        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    ei_sony_spresense_fs_close_sample_file();
    EiDevice.set_state(eiStateFinished);

    return retVal;
}