
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include "vl53l1_error_codes.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1
uint8_t _I2CBuffer[256];
uint8_t _ReadBuffer[128];

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	esp_err_t err = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);

	i2c_master_write_byte(cmd, dev << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, (index >> 8) & 0xff, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, index & 0xff, ACK_CHECK_EN);
	i2c_master_write(cmd, pdata, count, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE);
	i2c_cmd_link_delete(cmd);
	return err;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	esp_err_t err = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_write_byte(cmd, dev << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, (index >> 8) & 0xff, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, index & 0xff, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	
	i2c_master_write_byte(cmd, dev << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	for(uint32_t i = 0; i < count - 1; i++){
		i2c_master_read_byte(cmd, pdata + count, ACK_VAL);
	}
	i2c_master_read_byte(cmd, pdata + count, NACK_VAL);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE);
	i2c_cmd_link_delete(cmd);
	return err;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	_I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data;
	esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, dev, _I2CBuffer, 3, (I2C_TIME_OUT_BASE + 3 * I2C_TIME_OUT_BYTE)/portTICK_PERIOD_MS);
	if(err != 0){
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
	}
	return Status;
	//return VL53L1_WriteMulti(dev, index, &data,1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	_I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index & 0xFF;
    _I2CBuffer[2] = data >> 8;
	_I2CBuffer[3] = data & 0xFF;
	esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, dev, _I2CBuffer, 4, (I2C_TIME_OUT_BASE + 4 * I2C_TIME_OUT_BYTE)/portTICK_PERIOD_MS);
	if(err != 0){
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
	}
	return Status;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	_I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = (data >> 24) & 0xFF;
	_I2CBuffer[3] = (data >> 16) & 0xFF;
	_I2CBuffer[4] = (data >> 8) & 0xFF;
	_I2CBuffer[5] = data & 0xFF;
	esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, dev, _I2CBuffer, 6, (I2C_TIME_OUT_BASE + 6 * I2C_TIME_OUT_BYTE)/portTICK_PERIOD_MS);
	if(err != 0){
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
	}
	return Status;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	esp_err_t err;
	_I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index & 0xFF;
	err = i2c_master_write_read_device(I2C_NUM_0, dev, _I2CBuffer, 2, _ReadBuffer, 1, (I2C_TIME_OUT_BASE + 2 * I2C_TIME_OUT_BYTE)/portTICK_PERIOD_MS);
	if(err != 0){
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
	}
	uint8_t tmp = _ReadBuffer[0];
	*data = tmp;
	return Status;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	esp_err_t err;
	_I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index & 0xFF;
	err = i2c_master_write_read_device(I2C_NUM_0, dev, _I2CBuffer, 2, _ReadBuffer, 2, (I2C_TIME_OUT_BASE + 2 * I2C_TIME_OUT_BYTE)/portTICK_PERIOD_MS);
	if(err != 0){
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
	}
	uint8_t tmp = ((uint16_t)_ReadBuffer[0] << 8) + (uint16_t)_ReadBuffer[1];
	*data = tmp;
	return Status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	esp_err_t err;
	_I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index & 0xFF;
	err = i2c_master_write_read_device(I2C_NUM_0, dev, _I2CBuffer, 2, _ReadBuffer, 4, (I2C_TIME_OUT_BASE + 2 * I2C_TIME_OUT_BYTE)/portTICK_PERIOD_MS);
	if(err != 0){
		Status = VL53L1_ERROR_CONTROL_INTERFACE;
	}
	uint8_t tmp = ((uint16_t)_ReadBuffer[0] << 24) + ((uint16_t)_ReadBuffer[1] << 16) + ((uint16_t)_ReadBuffer[2] << 8) + (uint16_t)_ReadBuffer[3] ;
	*data = tmp;
	return Status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	vTaskDelay(wait_ms/portTICK_PERIOD_MS);
	return 0;
}
