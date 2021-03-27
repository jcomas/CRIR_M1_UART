/*******************************************************************
  CRIR M1 Library

Copyright (c) 2021 Josep Comas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


Example of packet (send):
<FE> <04> <00> <04> <00> <01> <64> <04>

<FE> -> Any address (8 bits)
<04> -> Function code  (8 bits)
<00> <04> -> Input Register 4 (16 bits), get temperature
<00> <01> -> Read 1 Word (16 bits)
<64> <04> -> CRC (16 bits)


Answer:
<FE> <04> <02> <30> <D4> <51> <9A>

<FE> -> Any address (8 bits)
<04> -> Function code  (8 bits)
<02> -> Length (number of bytes)
<D4> <51> -> Temperature (2 bytes)
<51> <9A> -> CRC (16 bits)


*******************************************************************/


#ifndef _CRIR_M1
    #define _CRIR_M1

    #if defined ARDUINO_ARCH_SAMD || defined ARDUINO_ARCH_SAM21D || defined ARDUINO_ARCH_ESP32 || defined ARDUINO_SAM_DUE || ARDUINO_ARCH_APOLLO3
        #undef USE_SOFTWARE_SERIAL
    #else
        #define USE_SOFTWARE_SERIAL
    #endif

    #include "Arduino.h"

    #ifdef USE_SOFTWARE_SERIAL       
        #include <SoftwareSerial.h>
    #endif

    #define CRIR_M1_LOG_LEVEL_NONE       (0)
    #define CRIR_M1_LOG_LEVEL_ERROR      (1)
    #define CRIR_M1_LOG_LEVEL_WARN       (2)
    #define CRIR_M1_LOG_LEVEL_INFO       (3)
    #define CRIR_M1_LOG_LEVEL_DEBUG      (4)
    #define CRIR_M1_LOG_LEVEL_VERBOSE    (5)

    #ifdef CORE_DEBUG_LEVEL
        #define CRIR_M1_LOG_LEVEL CORE_DEBUG_LEVEL
    #else
        #define CRIR_M1_LOG_LEVEL CRIR_M1_LOG_LEVEL_NONE
    #endif


    #if (CRIR_M1_LOG_LEVEL > CRIR_M1_LOG_LEVEL_NONE)

        /* Serial port for debug */

        // Uncomment if you use softwareserial for debug
        //#define CRIR_M1_DEBUG_SOFTWARE_SERIAL
        
        #ifdef CRIR_M1_DEBUG_SOFTWARE_SERIAL
            /* Modify if you use softwareserial for debug */
            #define CRIR_M1_DEBUG_SERIAL_RX 13
            #define CRIR_M1_DEBUG_SERIAL_TX 15
        #else
            /* Modify if you use hardware serial for debug */
            #define CRIR_M1_DEBUG_SERIAL Serial
        #endif

        /* Debug format */
        #define CRIR_M1_LOG(format, ...) CRIR_M1_DEBUG_SERIAL.printf(format, ##__VA_ARGS__)
    #else
        #define CRIR_M1_LOG(format, ...)
    #endif


    #define CRIR_M1_BAUDRATE 9600         // Device to CRIR M1 Serial baudrate (should not be changed)
    #define CRIR_M1_TIMEOUT  5            // Timeout for communication
    #define CRIR_M1_LEN_BUF_MSG  20       // Max length of buffer for communication with the sensor

    #define CRIR_M1_LEN_SN       10       // Length of serial number
    #define CRIR_M1_LEN_SOFTVER  10       // Length of software version    


    // Modbus
    #define MODBUS_ANY_ADDRESS                  0XFE    // CRIR M1 uses any address
    #define MODBUS_FUNC_READ_HOLDING_REGISTERS  0X03    // Read holding registers (HR)
    #define MODBUS_FUNC_READ_INPUT_REGISTERS    0x04    // Read input registers (IR)
    #define MODBUS_FUNC_PRESET_SINGLE_REGISTER  0x06    // Preset single register (SR)


    // Input registers for CRIR M1
    #define MODBUS_IR5             0x0004  // Temperature
    #define MODBUS_IR6             0x0005  // Meter Status
    #define MODBUS_IR7             0x0006  // Output Status
    #define MODBUS_IR8             0x0007  // Space CO2
    #define MODBUS_IR9             0x0008  // PWM Output
    #define MODBUS_IR10            0x0009  // Sensor Type ID High
    #define MODBUS_IR11            0x000A  // Sensor Type ID Low
    #define MODBUS_IR12            0x000B  // Memory Map version
    #define MODBUS_IR13            0x000C  // FW version Main.Sub
    #define MODBUS_IR14            0x000D  // Sensor ID High
    #define MODBUS_IR15            0x000E  // Sensor ID Low
    #define MODBUS_IR16            0x000F  // Serial Num.1
    #define MODBUS_IR17            0x0010  // Serial Num.2
    #define MODBUS_IR18            0x0011  // Serial Num.3
    #define MODBUS_IR19            0x0012  // Serial Num.4
    #define MODBUS_IR20            0x0013  // Serial Num.5


    // Holding registers for CRIR M1
    #define MODBUS_HR5             0x0004  // ABC Period
    #define MODBUS_HR6             0x0005  // User Acknowledgement Register
    #define MODBUS_HR7             0x0006  // User Special Command Register
    #define MODBUS_HR8             0x0007  // User Concentration


    // Meter status
    #define CRIR_M1_MASK_METER_OUT_OF_RANGE        0x0020   // Out of range
    #define CRIR_M1_MASK_METER_MEMORY_ERROR        0x0040   // Memory error


    // Output status
    #define CRIR_M1_MASK_OUTPUT_ALARM              0x0001   // Alarm output
    #define CRIR_M1_MASK_OUTPUT_PWM                0x0002   // PWM output


    // Calibration definitions
    #define CRIR_M1_CLEAR_CALIBRATION_COMPLETION   0x0000   // Clear the calibration completion flag
    #define CRIR_M1_START_USER_CALIBRATION         0x7C01   // Command to start user calibration
    #define CRIR_M1_CALIBRATION_COMPLETED          0x0001   // Calibration completed


    struct CRIR_M1_sensor {
        char sn[CRIR_M1_LEN_SN + 1];
        char softver[CRIR_M1_LEN_SOFTVER + 1];
        int16_t co2;
        int16_t temperature;
    };

    class CRIR_M1
    {
        public:
            CRIR_M1(Stream &serial);                                             // Initialize
            void get_serial_number(char sn[]);                                   // Get serial number
            void get_software_version(char softver[]);                           // Get software version
            int16_t get_co2();                                                   // Get CO2 value in ppm
            int16_t get_temperature();                                           // Get detector temperature in celsius degree
            int16_t get_ABC_period();                                            // Get ABC period in hours
            bool set_ABC_period(int16_t period);                                 // Set ABC period (4 - 4800 hours, 0 to disable)
            int16_t get_user_concentration();                                    // Get user concentration in ppm
            bool set_user_concentration(int16_t concentration);                  // Set user concentration in ppm
            int16_t get_user_acknowledgement();                                  // Get user acknowledgement
            bool set_user_acknowledgement(int16_t flag);                         // Set user acknowledgement
            bool set_user_special_command(int16_t command);                      // Set user special command
            int16_t get_meter_status();                                          // Get meter status
            int16_t get_output_status();                                         // Get output status
            int16_t get_PWM_output();                                            // Get PWM output
            int32_t get_sensor_type_ID();                                        // Get sensor type ID
            int32_t get_sensor_ID();                                             // Get sensor ID
            int16_t get_memory_map_version();                                    // Get memory map version

        private:
            Stream* mySerial;                                                    // Communication serial with the sensor
            uint8_t buf_msg[CRIR_M1_LEN_BUF_MSG];                                // Buffer for communication messages with the sensor

            void serial_write_bytes(uint8_t size);                               // Send bytes to sensor
            uint8_t serial_read_bytes(uint8_t max_bytes, int timeout_seconds);   // Read received bytes from sensor
            bool valid_response(uint8_t func, uint8_t nb);                       // Check if response is valid according to sent command
            bool valid_response_len(uint8_t func, uint8_t nb, uint8_t len);      // Check if response is valid according to sent command and checking expected total length
            void send_cmd(uint8_t func, uint16_t cmd, uint16_t value);           // Send command
            void print_buffer(uint8_t size);                                     // Show buffer in hex bytes
            void print_binary(int16_t number);                                   // Show number in bits
    };

#endif
