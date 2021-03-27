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

*******************************************************************/

#include "crir_m1.h"
#include "modbus_crc.h"

#if (CRIR_M1_LOG_LEVEL > CRIR_M1_LOG_LEVEL_NONE)
    #ifdef CRIR_M1_DEBUG_SOFTWARE_SERIAL
        SoftwareSerial CRIR_M1_DEBUG_SERIAL(CRIR_M1_DEBUG_SERIAL_RX, CRIR_M1_DEBUG_SERIAL_TX);
    #endif        
#endif

/* Initialize */
CRIR_M1::CRIR_M1(Stream &serial)
{
    mySerial = &serial;
}

/* Get serial number */
void CRIR_M1::get_serial_number(char sn[] ) {

    if (sn == NULL) {
        return;
    }

    strcpy(sn, "");

    // Ask serial number
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR16, 0x0005);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(15, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 15)) {

        strncat(sn, (const char *) &buf_msg[3], CRIR_M1_LEN_SN);
        CRIR_M1_LOG("DEBUG: Serial number: %s\n", sn);

    } else {
        CRIR_M1_LOG("DEBUG: Serial number not available!\n");
    }

}


/* Get software version */
void CRIR_M1::get_software_version(char softver[]) {

    if (softver == NULL) {
        return;
    }

    strcpy(softver, "");

    // Ask software version
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR13, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 7)) {
        snprintf(softver, CRIR_M1_LEN_SOFTVER, "%0u.%0u", buf_msg[3], buf_msg[4]);
        CRIR_M1_LOG("DEBUG: Software version: %s\n", softver);
    } else {
        CRIR_M1_LOG("DEBUG: Software version not available!\n");
    }
    
}


/* Get CO2 value in ppm */
int16_t CRIR_M1::get_co2() {

    int16_t co2 = 0;

    // Ask CO2 value
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR8, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 7)) {
        co2 = ((buf_msg[3] << 8) & 0xFF00) | (buf_msg[4] & 0x00FF);
        CRIR_M1_LOG("DEBUG: CO2 value = %d ppm\n", co2);
    } else {
        CRIR_M1_LOG("DEBUG: Error getting CO2 value!\n");
    }
    return co2;
}


/* Get temperature in celsius degree */
int16_t CRIR_M1::get_temperature() {

    int16_t temp = 0;

    // Ask temperature value
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR5, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 7)) {
        temp = (((buf_msg[3] * 256) + buf_msg[4]) / 100) - 100;
        CRIR_M1_LOG("DEBUG: Temperature value = %d C\n", temp);
    } else {
        CRIR_M1_LOG("DEBUG: Error getting temperature value!\n");
    }
    return temp;
}


/* Read ABC period */
int16_t CRIR_M1::get_ABC_period() {

    int16_t period = 0;

    // Ask ABC period
    send_cmd(MODBUS_FUNC_READ_HOLDING_REGISTERS, MODBUS_HR5, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_HOLDING_REGISTERS, nb, 7)) {
        period = ((buf_msg[3] << 8) & 0xFF00) | (buf_msg[4] & 0x00FF);
        CRIR_M1_LOG("DEBUG: ABC period = %d hours\n", period);
    } else {
        CRIR_M1_LOG("DEBUG: Error getting ABC period!\n");
    }
    return period;
}


/* Setup ABC period */
bool CRIR_M1::set_ABC_period(int16_t period) {
    uint8_t buf_msg_sent[8]; 
    bool result = false;

    if (period == 0 || (period >= 4 && period <= 4800)) {

        // Ask set ABC period
        send_cmd(MODBUS_FUNC_PRESET_SINGLE_REGISTER, MODBUS_HR5, period);

        // Save bytes sent
        memcpy(buf_msg_sent, buf_msg, 8);

        // Wait response
        memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
        serial_read_bytes(8, CRIR_M1_TIMEOUT);

        // Check response
        if (memcmp(buf_msg_sent, buf_msg, 8) == 0) {
            result = true;
            CRIR_M1_LOG("DEBUG: Successful setting of ABC period\n");
        } else {
            CRIR_M1_LOG("DEBUG: Error in setting of ABC period!\n");
        }

    } else {
        CRIR_M1_LOG("DEBUG: Invalid ABC period!\n");
    }

    return result;
}


/* Read user concentration */
int16_t CRIR_M1::get_user_concentration() {

    int16_t concentration = 0;

    // Ask ABC period
    send_cmd(MODBUS_FUNC_READ_HOLDING_REGISTERS, MODBUS_HR8, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_HOLDING_REGISTERS, nb, 7)) {
        concentration = ((buf_msg[3] << 8) & 0xFF00) | (buf_msg[4] & 0x00FF);
        CRIR_M1_LOG("DEBUG: User concentration = %d ppm\n", concentration);
    } else {
        CRIR_M1_LOG("DEBUG: Error getting user concentration!\n");
    }
    return concentration;
}


/* Setup user concentration */
bool CRIR_M1::set_user_concentration(int16_t concentration) {
    uint8_t buf_msg_sent[8]; 
    bool result = false;

    if (concentration >= 400 && concentration <= 2000) {

        // Ask set user concentration
        send_cmd(MODBUS_FUNC_PRESET_SINGLE_REGISTER, MODBUS_HR8, concentration);

        // Save bytes sent
        memcpy(buf_msg_sent, buf_msg, 8);

        // Wait response
        memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
        serial_read_bytes(8, CRIR_M1_TIMEOUT);

        // Check response
        if (memcmp(buf_msg_sent, buf_msg, 8) == 0) {
            result = true;
            CRIR_M1_LOG("DEBUG: Successful setting user concentration\n");
        } else {
            CRIR_M1_LOG("DEBUG: Error in setting user concentration!\n");
        }

    } else {
        CRIR_M1_LOG("DEBUG: Invalid user concentration!\n");
    }

    return result;
}


/* Read user acknowledgement */
int16_t CRIR_M1::get_user_acknowledgement() {

    int16_t flag = 0;

    // Ask user acknowledgement
    send_cmd(MODBUS_FUNC_READ_HOLDING_REGISTERS, MODBUS_HR6, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_HOLDING_REGISTERS, nb, 7)) {
        flag = ((buf_msg[3] << 8) & 0xFF00) | (buf_msg[4] & 0x00FF);
        CRIR_M1_LOG("DEBUG: User acknowledgement flag = %d\n", flag);
    } else {
        CRIR_M1_LOG("DEBUG: Error getting user acknowledgement flag!\n");
    }
    return flag;
}


/* Setup user acknowledgement */
bool CRIR_M1::set_user_acknowledgement(int16_t flag) {
    uint8_t buf_msg_sent[8]; 
    bool result = false;

    // Ask set user acknowledgement
    send_cmd(MODBUS_FUNC_PRESET_SINGLE_REGISTER, MODBUS_HR6, flag);

    // Save bytes sent
    memcpy(buf_msg_sent, buf_msg, 8);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    serial_read_bytes(8, CRIR_M1_TIMEOUT);

    // Check response
    if (memcmp(buf_msg_sent, buf_msg, 8) == 0) {
        result = true;
        CRIR_M1_LOG("DEBUG: Successful setting user acknowledgement\n");
    } else {
        CRIR_M1_LOG("DEBUG: Error in setting user acknowledgement!\n");
    }

    return result;
}


/* Setup user special command */
bool CRIR_M1::set_user_special_command(int16_t command) {
    uint8_t buf_msg_sent[8]; 
    bool result = false;

    // Ask set user special command
    send_cmd(MODBUS_FUNC_PRESET_SINGLE_REGISTER, MODBUS_HR7, command);

    // Save bytes sent
    memcpy(buf_msg_sent, buf_msg, 8);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    serial_read_bytes(8, CRIR_M1_TIMEOUT);

    // Check response
    if (memcmp(buf_msg_sent, buf_msg, 8) == 0) {
        result = true;
        CRIR_M1_LOG("DEBUG: Successful setting user special command\n");
    } else {
        CRIR_M1_LOG("DEBUG: Error in setting user special command!\n");
    }

    return result;
}


/* Read meter status */
int16_t CRIR_M1::get_meter_status() {

    int16_t status = 0;

    // Ask meter status
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR6, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 7)) {
        status = ((buf_msg[3] << 8) & 0xFF00) | (buf_msg[4] & 0x00FF);
        CRIR_M1_LOG("DEBUG: Meter status = b");
#if (CRIR_M1_LOG_LEVEL > CRIR_M1_LOG_LEVEL_NONE)
        print_binary(status);
#endif
        CRIR_M1_LOG("\n");        
    } else {
        CRIR_M1_LOG("DEBUG: Error getting meter status!\n");
    }
    return status;
}


/* Read output status */
int16_t CRIR_M1::get_output_status() {

    int16_t status = 0;

    // Ask output status
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR7, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 7)) {
        status = ((buf_msg[3] << 8) & 0xFF00) | (buf_msg[4] & 0x00FF);
        CRIR_M1_LOG("DEBUG: Output status = b");
#if (CRIR_M1_LOG_LEVEL > CRIR_M1_LOG_LEVEL_NONE)
        print_binary(status);
#endif
        CRIR_M1_LOG("\n");
    } else {
        CRIR_M1_LOG("DEBUG: Error getting output status!\n");
    }
    return status;
}


/* Read PWM output */
int16_t CRIR_M1::get_PWM_output() {

    int16_t pwm = 0;

    // Ask PWM output
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR9, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 7)) {
        pwm = ((buf_msg[3] << 8) & 0xFF00) | (buf_msg[4] & 0x00FF);
        CRIR_M1_LOG("DEBUG: PWM output = %d\n", pwm);
    } else {
        CRIR_M1_LOG("DEBUG: Error getting PWM output!\n");
    }
    return pwm;
}


/* Read sensor type ID */
int32_t CRIR_M1::get_sensor_type_ID() {

    int32_t sensorType = 0;

    // Ask sensor type ID
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR10, 0x0002);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(9, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 9)) {
        sensorType = ((buf_msg[3] << 24) & 0xFF000000) | ((buf_msg[4] << 16) & 0x00FF0000) | ((buf_msg[5] << 8) & 0x0000FF00) | (buf_msg[6] & 0x000000FF);
        CRIR_M1_LOG("DEBUG: Sensor type ID = 0x%08x\n", sensorType);
    } else {
        CRIR_M1_LOG("DEBUG: Error getting sensor type ID!\n");
    }
    return sensorType;
}


/* Read sensor ID */
int32_t CRIR_M1::get_sensor_ID() {

    int32_t sensorID = 0;

    // Ask sensor ID
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR14, 0x0002);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(9, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 9)) {
        sensorID = ((buf_msg[3] << 24) & 0xFF000000) | ((buf_msg[4] << 16) & 0x00FF0000) | ((buf_msg[5] << 8) & 0x0000FF00) | (buf_msg[6] & 0x000000FF);
        CRIR_M1_LOG("DEBUG: Sensor ID = 0x%08x\n", sensorID);
    } else {
        CRIR_M1_LOG("DEBUG: Error getting sensor ID!\n");
    }
    return sensorID;
}


/* Read memory map version */
int16_t CRIR_M1::get_memory_map_version() {

    int16_t mmVersion = 0;

    // Ask memory map version
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR9, 0x0001);

    // Wait response
    memset(buf_msg, 0, CRIR_M1_LEN_BUF_MSG);
    uint8_t nb = serial_read_bytes(7, CRIR_M1_TIMEOUT);

    // Check response and get data
    if (valid_response_len(MODBUS_FUNC_READ_INPUT_REGISTERS, nb, 7)) {
        mmVersion = ((buf_msg[3] << 8) & 0xFF00) | (buf_msg[4] & 0x00FF);
        CRIR_M1_LOG("DEBUG: Memory map version = %04x\n", mmVersion);
    } else {
        CRIR_M1_LOG("DEBUG: Error getting memory map version!\n");
    }
    return mmVersion;
}


/* Check valid response and length of received message */
bool CRIR_M1::valid_response_len(uint8_t func, uint8_t nb, uint8_t len) {
    bool result = false;

    if (nb == len) {
        result = valid_response(func, nb);
    } else {
        CRIR_M1_LOG("DEBUG: Unexpected length\n");
    }

    return result;
}


/* Check if it is a valid message response of the sensor */
bool CRIR_M1::valid_response(uint8_t func, uint8_t nb) {

    uint16_t crc16;
    bool result = false;

    if (nb >= 7) {
        crc16 = modbus_CRC16(buf_msg, nb-2);
        if ((buf_msg[nb-2] == (crc16 & 0x00FF)) && (buf_msg[nb-1] == ((crc16 >> 8) & 0x00FF))) {

            if (buf_msg[0] == MODBUS_ANY_ADDRESS && (buf_msg[1] == MODBUS_FUNC_READ_HOLDING_REGISTERS || buf_msg[1] == MODBUS_FUNC_READ_INPUT_REGISTERS) && buf_msg[2] == nb-5) {
                CRIR_M1_LOG("DEBUG: Valid response\n");
                result = true;

            } /* else if (buf_msg[0] == CM1106_MSG_NAK && nb == 4) {
                CRIR_M1_LOG("DEBUG: Response with error 0x%02x\n", buf_msg[2]);
                // error 0x02 = cmd not recognised, invalid checksum...
                // If invalid length then no response.
            } */

        } else {
            CRIR_M1_LOG("DEBUG: Checksum/length is invalid\n");
        }

    } else {
        CRIR_M1_LOG("DEBUG: Invalid length\n");
    }
    
    return result;
}


/* Send command */
void CRIR_M1::send_cmd( uint8_t func, uint16_t cmd, uint16_t value) {

    uint16_t crc16;

    if (((func == MODBUS_FUNC_READ_HOLDING_REGISTERS || func == MODBUS_FUNC_READ_INPUT_REGISTERS) && value >= 1) || (func == MODBUS_FUNC_PRESET_SINGLE_REGISTER)) {
        buf_msg[0] = MODBUS_ANY_ADDRESS;                // Address
        buf_msg[1] = func;                              // Function
        buf_msg[2] = (cmd >> 8) & 0x00FF;               // High-input register
        buf_msg[3] = cmd & 0x00FF;                      // Low-input register
        buf_msg[4] = (value >> 8) & 0x00FF;             // High-word to read or setup
        buf_msg[5] = value & 0x00FF;                    // Low-word to read or setup
        crc16 = modbus_CRC16(buf_msg, 6);
        //Serial.printf("CRC value: 0x%04x\n", crc16);
        buf_msg[6] = crc16 & 0x00FF;
        buf_msg[7] = (crc16 >> 8) & 0x00FF;
        serial_write_bytes(8);        
    }
}


/* Send bytes to sensor */
void CRIR_M1::serial_write_bytes(uint8_t size) {

#if (CRIR_M1_LOG_LEVEL > CRIR_M1_LOG_LEVEL_NONE)     
    CRIR_M1_LOG("DEBUG: Bytes to send => ");
    print_buffer(size);
#endif

    mySerial->write(buf_msg, size);
    mySerial->flush();
}


/* Read answer of sensor */
uint8_t CRIR_M1::serial_read_bytes(uint8_t max_bytes, int timeout_seconds) {

    time_t start_t, end_t;
    //double diff_t;
    time(&start_t); end_t = start_t;
    bool readed = false;

    uint8_t nb = 0;
    if (max_bytes > 0 && timeout_seconds > 0) {

        CRIR_M1_LOG("DEBUG: Bytes received => ");

        while ((difftime(end_t, start_t) <= timeout_seconds) && !readed) {
            if(mySerial->available()) {
                nb = mySerial->readBytes(buf_msg, max_bytes);
                readed = true;
            }            
            time(&end_t);
        }

#if (CRIR_M1_LOG_LEVEL > CRIR_M1_LOG_LEVEL_NONE)
        print_buffer(nb);
#endif

    } else {
        CRIR_M1_LOG("DEBUG: Invalid parameters!\n");
    }

    return nb;
}


/* Show buffer in hex bytes */
void CRIR_M1::print_buffer(uint8_t size) {

    for (int i = 0; i < size; i++) {
        CRIR_M1_LOG("0x%02x ", buf_msg[i]);
    }
    CRIR_M1_LOG("(%u bytes)\n", size);
}


/* Show number in binary */
void CRIR_M1::print_binary(int16_t number) {
    int16_t k;

    for (int8_t c = 15; c >= 0; c--)
    {
        k = number >> c;

        if (k & 1)
            CRIR_M1_LOG("1");
        else
            CRIR_M1_LOG("0");
    }
}
