#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <sys/time.h>
#include <string.h>

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "bsec_integration.h"




#define I2C_ACK      0x0
#define I2C_NACK     0x1

#define I2C_BUS      0
#define I2C_SCL      14
#define I2C_SDA      13
#define I2C_FREQ     100000

#define ACK_CHECK_EN 1



int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr) << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);

    if (reg_addr){
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    }

    if (reg_data_ptr){
        i2c_master_write(cmd, reg_data_ptr, data_len, ACK_CHECK_EN);        
    }

    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    return res;
}


int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    if (!data_len){
        return 1;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    if (!reg_data_ptr)
        i2c_master_stop(cmd);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (data_len > 1) i2c_master_read(cmd, reg_data_ptr, data_len-1, I2C_ACK);
    i2c_master_read_byte(cmd, reg_data_ptr + data_len-1, I2C_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t res = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return res;
}


void sleep_funct(uint32_t t_ms)
{
    vTaskDelay(t_ms / portTICK_RATE_MS);
}


int64_t get_timestamp_us()
{
    struct timeval time;
    gettimeofday(&time,0);
    return time.tv_sec*1e6 + time.tv_usec;
}


void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,float voc, float siaq, int siaq_accuracy)
{
    printf("temp: %.2f | hum: %.0f | pres: %.0f | scaled iaq: %.0f | gas : %.0f | BVOC (ppm): %.3f | SIAQ: %.0f | ACC: %d\n",temperature,humidity,pressure,iaq,gas,voc,siaq,siaq_accuracy);
    
}
    

uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{

    return 0;
}


void state_save(const uint8_t *state_buffer, uint32_t length)
{

}
 

uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{

    return 0;
}


void app_main(void)
{
   
    return_values_init ret;
    
i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 13,
        .scl_io_num = 14,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000};
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    /* Call to the function which initializes the BSEC library 
    * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, sleep_funct, state_load, config_load);

        if (ret.bme680_status)
    {
        printf("Could not intialize BME680\n");
    }
    else if (ret.bsec_status)
    {
        printf("Could not intialize BSEC library \n");
    }
    
    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(sleep_funct, get_timestamp_us, output_ready, state_save, 10000);
    
}
