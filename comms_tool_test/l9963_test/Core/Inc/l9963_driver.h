
#ifndef L9963_DRIVER_H
#define L9963_DRIVER_H

#include "stm32f1xx_hal.h"
#include "usart.h"
#include "stdint.h"
#include "stdbool.h"


extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;

void l9963_data_loop(void);
void l9963_init();
bool l9963_getVoltages(float volt_buf[]);
bool l9963_getTempCoulomb(float measure_buf[]);
bool l9963_runDiagnose(void);
bool l9963_setCellBalance(uint32_t enable_balance_bit_mask);


static bool l9963_transmission_wrapper(bool write_true, bool is_broadcast, uint8_t address, uint32_t data_write, uint32_t data_arr_ptr[], bool error);
static bool l9963_config_wrapper(void);


static uint8_t calc_crc(uint64_t data);
static bool decipherMessage(uint32_t *data_buf_ptr);
static void assembleMessage(bool PA, bool RWB, uint8_t dev_id, uint8_t address, uint8_t GSW, uint32_t data_write);


struct registers_content {
    uint32_t dev_gen_cfg;
    uint32_t bal_1;
};

struct frame_data{
    bool PA; //1 is master, 0 is slave
    bool burst; //1 is burst frame, 0 is not
    uint8_t dev_id;

    uint8_t burst_num; // this of for burst only
    uint8_t address_feedback; //register address

    uint32_t data_read; // exists for both burst and single access

    uint8_t status_word; //0 is no fault
    uint8_t crc;
};

//uint8_t[5] *getSampleMessage(void);
#endif L9963_DRIVER_H

