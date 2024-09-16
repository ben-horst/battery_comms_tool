
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include "l9963_driver.h"

#define DEBUG_PRINT

#define Burst_0x78_FL 18
#define Burst_0x7A_FL 13
#define Burst_0x7B_FL 14


#define DEV_GEN_CFG 0x1
#define FASTCH_BALUV 0x2
#define BAL_1 0x3
#define BAL_2 0x4
#define BAL_3 0x5
#define VCELLS_EN 0x1C
#define BURST_0x78 0x78
#define BURST_0x7A 0x7A
#define BURST_0x7B 0x7B
#define ADVC_CONV 0xD
#define NCYCLE_PROG_1 0xE
#define FAULTMASK 0x1D
#define FAULTMASK2 0x1E
#define FAULTS_1 0x3C
#define FAULTS_2 0x3D
#define VCELL_THRESH_UV_OV 0xB
#define VBAT_SUM_TH 0xC
#define BALCELL14_7ACT 0x10
#define BALCELL6_1ACT 0x11


#define VCELL_RES 89e-6
#define VCELL_RES_OVUV 22.784e-3
#define VPACK_RES_OVUV 364.544e-3
#define ICELL_RES 13.33e-3


uint8_t spi_buf[5];
uint8_t msg_buf[5] = {0};


uint64_t message_non_crc;
uint8_t crc_value;
uint64_t crc_expanded;
uint64_t message_crc;
uint8_t spi_receive[5] = {0};
uint64_t message_complete = 0;
uint8_t CRC_received = 0;
uint8_t CRC_receive_calc = 0;

uint32_t default_buf = 0; //there for no reason as of now

char uart_buf[50];
int uart_buf_len = 0;
struct frame_data inc_msg;

struct registers_content reg_data_read;


bool requested_burst = false;



/**
  * @brief L9963 Initialization Function
  * @param None
  * @retval None
  */
void l9963_init(void) {

    // CS pin should default high
    HAL_GPIO_WritePin(L9963_CS_GPIO_Port, L9963_CS_Pin, GPIO_PIN_SET);
    uart_buf_len = sprintf(uart_buf, "Sending Config frames\n");
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);

    if (l9963_config_wrapper()) {
#ifdef DEBUG_PRINT
        uart_buf_len = sprintf(uart_buf, "Config_success \n");
        HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
#endif
    } else {
#ifdef DEBUG_PRINT
        uart_buf_len = sprintf(uart_buf, "Config_fail \n");
        HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
#endif
    }
}

/***
 * Pulls the voltage data from the L9963E
 * @param volt_buf
 * @return True if voltages could be read
 */
bool l9963_getVoltages(float *volt_buf)
{
    uint32_t data_array[Burst_0x78_FL] = {0};
    //TODO it is also containing current data... How to split this?
    l9963_transmission_wrapper(true, false, ADVC_CONV, 0x8000, data_array, false);
    l9963_transmission_wrapper(false, false, ADVC_CONV, 0x0000, data_array, false);
//maybe some delay needed here?
    HAL_Delay(1);

    //Voltage gets converted within this function
    if(l9963_transmission_wrapper(false, false, BURST_0x78, 0x0, data_array, false))
    {
        uint32_t volt_preVal = 0;
        for (int i = 0; i < Burst_0x78_FL; ++i)
        {
            if (i < 14)
            {
                volt_buf[i + 1] = (float) (data_array[i] & 0xFFFF) * VCELL_RES;
            }
            else if (i == 14)
            {
                volt_preVal = data_array[i] << 2;
            }
            else if (i == 15)
            {
                volt_preVal |= data_array[i] >> 16;
                volt_buf[0] = (float) volt_preVal * VCELL_RES;
            }
            else if (i == 17)
            {
                volt_buf[15] = (float)((int32_t)(data_array[i] | ((data_array[i] & (1<<17)) ? 0xFFFC0000u : 0))) * ICELL_RES;
                //TODO connect a current shunt to measure current --> not connected right now
            }
        }
#ifdef  DEBUG_PRINT
        uart_buf_len = sprintf(uart_buf, "Finished\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
#endif
        return true;
    }
    else
    {
        return false;
    }
}

/****
 * Returns temperature and coulomb data
 * @param measure_buf
 * @return true if data was acquired
 */
bool l9963_getTempCoulomb(float measure_buf[])
{
    uint32_t data_array[Burst_0x7B_FL] = {0};
    l9963_transmission_wrapper(true, false, ADVC_CONV, 0x8000, data_array, false);
    l9963_transmission_wrapper(false, false, ADVC_CONV, 0x0000, data_array, false);
//maybe some delay needed here?
    HAL_Delay(1);

    //Temp and Coulomb get converted within this function
    if(l9963_transmission_wrapper(false, false, BURST_0x7B, 0x0, data_array, false))
    {
        for (int i = 0; i < Burst_0x7B_FL; ++i)
        {
            if (i < 5)
            {
                //measure_buf[]
            }
            else if (i <13)
            {
                measure_buf[i] = (data_array[i]&0xFFF);
            }
            else if (i == 14)
            {
                measure_buf[i] = (data_array[i] & 0x7F)*1.3828;
            }
        }
    }

}
/***
 * Analyzes the fault register of the l9963 via burst
 * @return true if no fault detected
 */
bool l9963_runDiagnose(void)
{
    uint32_t data_array[Burst_0x7A_FL] = {0};
    l9963_transmission_wrapper(true, false, ADVC_CONV, 0x8000, data_array, false);
    l9963_transmission_wrapper(false, false, ADVC_CONV, 0x0000, data_array, false);
//maybe some delay needed here?
    HAL_Delay(1);

    //diagnose data is read within this function
    if(l9963_transmission_wrapper(false, false, BURST_0x7A, 0x0, data_array, false))
    {
        //do_sth.
    }
}

/***
 * Set BMS to balance mode
 * Discharges select cells, zero indexed. Resistor slowly discharges
 * Setting to 0 turns off balancing in general
 * We assume only connected cells will be asked for
 * @param enable_balance_bit_mask
 * @return true if balance mode could be set
 */
bool l9963_setCellBalance(uint32_t enable_balance_bit_mask)
{
    //TODO actually map the bits for balancing. For now turns on cells 1 and 13
    if  (!enable_balance_bit_mask)
    {
        l9963_transmission_wrapper(true, false, BALCELL14_7ACT, 0, (uint32_t *)default_buf, false);
        l9963_transmission_wrapper(true, false, BALCELL6_1ACT, 0, (uint32_t *)default_buf, false);
        //we disable all balancing
        return(l9963_transmission_wrapper(true, false, BAL_1, 0x24000, (uint32_t *)default_buf, false));
    }
    else
    {
        uint32_t bal_6_1_mask = 0;
        uint32_t bal_14_7_mask = 0;
        for (int i = 0; i < 6; ++i)
        {
            if ((enable_balance_bit_mask >> i)&1U)
            {
                bal_6_1_mask |= (1U << 5+i*2);
            }
        }
        for (int i = 0; i < 8; ++i)
        {
            if ((enable_balance_bit_mask >> i+6)&1U)
            {
                bal_14_7_mask |= (1U << 1+i*2);
            }
        }

        l9963_transmission_wrapper(true, false, BAL_1, 0x24000, (uint32_t *)default_buf, false);
        l9963_transmission_wrapper(true, false, BALCELL14_7ACT, bal_14_7_mask, (uint32_t *)default_buf, false);
        l9963_transmission_wrapper(true, false, BALCELL6_1ACT, bal_6_1_mask, (uint32_t *)default_buf, false);
        l9963_transmission_wrapper(true, false, BAL_1, 0x28000, (uint32_t *)default_buf, false);
        return true;
    }
}



/***
 * Wrapper that sets the default parameters for the L9963
 * @return True if the registers are set
 */
static bool l9963_config_wrapper(void)
{
    //We try 2 times here
        //wakeup
        l9963_transmission_wrapper(false, false, DEV_GEN_CFG, 0, (uint32_t *)default_buf, false);
        l9963_transmission_wrapper(false, false, DEV_GEN_CFG, 0,(uint32_t *)default_buf,  false);
        //broadcast command to assign the id
        l9963_transmission_wrapper(true, true, DEV_GEN_CFG, 0x2000, (uint32_t *)default_buf, false);

        //read the NVM at least once, triggers error otherwise
        l9963_transmission_wrapper(true, false, BAL_3, 0x10000,(uint32_t *)default_buf,  false);
        //stop the read, do sth else in between
        l9963_transmission_wrapper(true, false, BAL_3, 0,(uint32_t *)default_buf,  false);
        //disable the timeout register, for testing purposes.
        l9963_transmission_wrapper(true, false, BAL_1, 0x20000,(uint32_t *)default_buf,  false);


        //read the timeout config register
        l9963_transmission_wrapper(false, false, BAL_1, 0, (uint32_t *)default_buf, false);
        //read the device config register
        l9963_transmission_wrapper(false, false, DEV_GEN_CFG, 0,(uint32_t *)default_buf,  false);
        // lock the device config registers via broadcast
        l9963_transmission_wrapper(true, true, DEV_GEN_CFG, 0, (uint32_t *)default_buf, false);

        //force an error reset (toggle the bit)
       // l9963_transmission_wrapper(true, false, DEV_GEN_CFG, 0x2001,(uint32_t *)default_buf,  false);
       // l9963_transmission_wrapper(true, false, DEV_GEN_CFG, 0x2000,(uint32_t *)default_buf,  false);


        // * We disable all config for now. The GUI is also NOT doing this!
        //this sets the cycle to 011, bit offset is 1
        //l9963_transmission_wrapper(true, false, ADVC_CONV, 0x6,(uint32_t *)default_buf,  false);
        //set the ignore term (ncycle) for the voltage to 011, bit offset is 10, set cyclic conversion to 1 (enable register updates)
        //l9963_transmission_wrapper(true, false, NCYCLE_PROG_1, 0xC10,(uint32_t *)default_buf,  false);
        //Enable Measurements on a cyclic basis, still leaving the cycle the same
        //l9963_transmission_wrapper(true, false, ADVC_CONV, 0x2006,(uint32_t *)default_buf,  false);

        //Battery_cell-overvoltage settings: set OV to 4.2 V (B8 with conversion), UV to 3V (83), upper two bits empty
        l9963_transmission_wrapper(true, false, VCELL_THRESH_UV_OV, 0xB883,(uint32_t *)default_buf,  false);
        //Battery-pack - OV for 16.8V at 0x2E, UV at 13V with 0x24
        l9963_transmission_wrapper(true, false, VBAT_SUM_TH, 0x2E24,(uint32_t *)default_buf,  false);



         // Setting up cells for testing. This enables all cells
        l9963_transmission_wrapper(true, false, VCELLS_EN, 0x3fff,(uint32_t *)default_buf,  false);

        //set balancing to manual mode. as used in our case
        l9963_transmission_wrapper(true, false, BAL_2, 0x10000,(uint32_t *)default_buf,  false);

        //we write the data here to check if it has this config.
    return l9963_transmission_wrapper(true, false, DEV_GEN_CFG, 0x2000, (uint32_t *)default_buf, false);

}


/***
 * Function that requests data whenever called
 */
void l9963_data_loop(void)
{
    //force an error reset (toggle the bit)
   // l9963_transmission_wrapper(true, false, DEV_GEN_CFG, 0x2001,(uint32_t *)default_buf,  false);
    //l9963_transmission_wrapper(true, false, DEV_GEN_CFG, 0x2000,(uint32_t *)default_buf,  false);


   // l9963_transmission_wrapper(false, false, DEV_GEN_CFG, 0x0,(uint32_t *)default_buf,  false);
   // l9963_transmission_wrapper(false, false, VCELLS_EN, 0x0, (uint32_t *)default_buf, false);
    l9963_transmission_wrapper(false, false, FAULTMASK, 0x0, (uint32_t *)default_buf, false);
    l9963_transmission_wrapper(false, false, FAULTMASK2, 0x0, (uint32_t *)default_buf, false);
    l9963_transmission_wrapper(false, false, FAULTS_1, 0x0, (uint32_t *)default_buf, false);
    l9963_transmission_wrapper(false, false, FAULTS_2, 0x0, (uint32_t *)default_buf, false);

}


/***
 * Function that handles the transmission to l9963
 * @param write_true Do we want to write (1) or read(0)
 * @param is_broadcast Not relevant for single bms
 * @param register_address Which address to write to
 * @param data_write What data to write
 * @param data_arr_ptr We trust the user inputs an appropriately sized array
 * @param error Communicate an error to the bms
 * @return if we had an error reading a data bit this will return false
 */
static bool l9963_transmission_wrapper(bool write_true, bool is_broadcast, uint8_t register_address, uint32_t data_write, uint32_t data_arr_ptr[], bool error)
{
    uint32_t retBuff = 0;
    uint32_t volt_preVal = 0;

    bool retVal = true;
    assembleMessage(1, write_true, (!is_broadcast), register_address, 0, data_write);
#ifdef DEBUG_PRINT
    uart_buf_len = sprintf(uart_buf, "Sending: % " PRIx64, message_crc);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
    uart_buf_len = sprintf(uart_buf, "Address: %X ", register_address);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
    uart_buf_len = sprintf(uart_buf, "Data: %X \n", data_write);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
#endif
    HAL_GPIO_WritePin(L9963_CS_GPIO_Port, L9963_CS_Pin, GPIO_PIN_RESET); //select slave
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) spi_buf, (uint8_t *) spi_receive, 5, 100);
    HAL_GPIO_WritePin(L9963_CS_GPIO_Port, L9963_CS_Pin, GPIO_PIN_SET); //deselect slave
    decipherMessage((uint32_t *) data_arr_ptr);

    HAL_GPIO_WritePin(L9963_CS_GPIO_Port, L9963_CS_Pin, GPIO_PIN_RESET); //select slave
    //TODO debug why but warning is to NEVER write to register 0, causes unexpected frame
    //TODO at which level shall the conversion happen? (type of input array,...)
    //TODO clean this function up
    switch (register_address)
    {
            case BURST_0x78:
                assembleMessage(1, 0, 1, 0x1, 0, 0);
                for (int i = 0; i < Burst_0x78_FL; ++i)
                {
                    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) spi_buf, (uint8_t *) spi_receive, 5, 100);
                if (!decipherMessage(&data_arr_ptr[i])){ return false;} // we are only interested in the first 16, later not interesting data here
                }
                    break;
            case BURST_0x7A:
                assembleMessage(1, 0, 1, 0x1, 0, 0);
                for (int i = 0; i < Burst_0x7A_FL; ++i)
                {
                    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) spi_buf, (uint8_t *) spi_receive, 5, 100);
                    if (!decipherMessage(&data_arr_ptr[i])) { retVal = false; }
                }
                break;
            case BURST_0x7B:
                assembleMessage(1, 0, 1, 0x1, 0, 0);
                for (int i = 0; i < Burst_0x7B_FL; ++i) {
                    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) spi_buf, (uint8_t *) spi_receive, 5, 100);
                    if (!decipherMessage(&data_arr_ptr[i])) { retVal = false; }
                }
                break;

            default:
                HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) spi_buf, (uint8_t *) spi_receive, 5, 100);
                if (!decipherMessage(&data_arr_ptr[0])) { retVal = false; }
                break;
    }
    HAL_GPIO_WritePin(L9963_CS_GPIO_Port, L9963_CS_Pin, GPIO_PIN_SET); //deselect slave

    if (write_true)
    {
        if (data_arr_ptr[0] != data_write)
        {retVal = false;
#ifdef DEBUG_PRINT
            uart_buf_len = sprintf(uart_buf, "CommandFailed\n");
            HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
#endif
        }
        else
        {
#ifdef DEBUG_PRINT
            uart_buf_len = sprintf(uart_buf, "CommandAccepted\n");
            HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
#endif
        }
    }
    return retVal;
}


/***
 * Creates a message from given input parameters
 * @param PA
 * @param RWB
 * @param dev_id
 * @param address
 * @param GSW
 * @param data_write
 */
static void assembleMessage(bool PA, bool RWB, uint8_t dev_id, uint8_t address, uint8_t GSW, uint32_t data_write )
{
    message_non_crc = ((uint64_t) PA <<39) | ((uint64_t) RWB <<38) | ((uint64_t) dev_id <<33) | ((uint64_t) address <<26) | ((uint64_t) GSW <<24)| ((uint64_t) data_write <<6);

    crc_value = calc_crc(message_non_crc);
    crc_expanded = crc_value;
    message_crc = message_non_crc | (crc_expanded);

    for (int i = 0; i < 5; i++)
    {
        spi_buf[4-i] = (message_crc >> i*8) & 0xFF;
    }
}
/***
 * Calculates the CRC based on 40 bits of data input. Note that the 6 lsb shall be 0
 * @param data
 * @return
 */
static uint8_t calc_crc(uint64_t data)
{
    //data <<= 6; // expand the data, shift left for 6 bits, adds 0b000000num...
    uint8_t crc = 0x31; // start with the seed
    for (int i = 40; i--;)
    {
        unsigned char c = (data >> i) & 1; //we start with first bit (40 shifted to right)
        bool bit = crc & 0x20;//we check if the 6th bit of the CRC is set
        crc = (crc << 1) | c; //we shift crc 1 left and set if either is true combined with data
        if (bit)
        {
            crc ^= 0x19; // poly, if 6th bit of crc is true then we update crc (XOR?)
        }
    }
    return crc & 0x3f; // only 6 bits
}

/***
 * Parses the message received
 * @return True if the message CRC was correct and message received
 */
static bool decipherMessage(uint32_t *data_buf_ptr)
{
    message_complete = 0;
    for (int i = 0; i < 5; i++)
    {
        message_complete |= ((uint64_t) spi_receive[4 - i] << i * 8);
    }
    CRC_received = message_complete & (uint64_t) 0x3F;

    uint64_t message_complete_shifted = message_complete >> 6;
    message_complete_shifted = message_complete_shifted << 6;
    CRC_receive_calc = calc_crc(message_complete_shifted);
    if (message_complete == 0ULL)
    {
#ifdef DEBUG_PRINT
        uart_buf_len = sprintf(uart_buf, "No message received\n");
        HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
#endif
        return false;
    }
    if (CRC_receive_calc != CRC_received)
    {
#ifdef DEBUG_PRINT
        uart_buf_len = sprintf(uart_buf, "Wrong CRC\n");
        HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
#endif
        return false;
    }
    //TODO: This is legacy, should be writing to pointer addresses in future
    inc_msg.PA = (message_complete >> 39) & 0b1; // 1 bit
    inc_msg.dev_id = (message_complete >> 33) & 0x1F; //5 bits
    inc_msg.burst = (message_complete >> 38) & 0b1; // 1 bit
    inc_msg.address_feedback = (message_complete >> 26) & 0x7F; // 7 bit
    inc_msg.status_word = (message_complete >> 24) & 0b11; //2 bit
    inc_msg.data_read = (message_complete >> 6) & 0x3FFFF; //18 bits
    *data_buf_ptr = inc_msg.data_read;
    inc_msg.crc = CRC_received;
    if ((inc_msg.address_feedback >> 5) ==0b11)
    {
        if ((inc_msg.address_feedback == 0x78) || (inc_msg.address_feedback == 0x7A) ||
            (inc_msg.address_feedback == 0x7B))
        {
            inc_msg.burst_num = 1;
        }
        else
        {
           inc_msg.burst_num = inc_msg.address_feedback & 0x1F;
        }
        //data_buffer[inc_msg.burst_num] = inc_msg.data_read;
    }
    else
    {
        inc_msg.burst_num = 0;
    }


#ifdef DEBUG_PRINT
    uart_buf_len = sprintf(uart_buf, "FullMSG: % " PRIx64, message_complete);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
    uart_buf_len = sprintf(uart_buf, "IsMaster: %X ", (unsigned int) inc_msg.PA);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
    uart_buf_len = sprintf(uart_buf, "IsWrite: %X ", (unsigned int) inc_msg.burst);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
    uart_buf_len = sprintf(uart_buf, "dev_id: %X ", (unsigned int) inc_msg.dev_id);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
    uart_buf_len = sprintf(uart_buf, "address: %X ", (unsigned int) inc_msg.address_feedback);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
    if (inc_msg.burst_num)
    {
        uart_buf_len = sprintf(uart_buf, "Burst Frame: %d ", (unsigned int) inc_msg.burst_num);
        HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
    }

    uart_buf_len = sprintf(uart_buf, "GSW: %X ", (unsigned int) inc_msg.status_word);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
    uart_buf_len = sprintf(uart_buf, "Data: %X ", (unsigned int) *data_buf_ptr);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
    uart_buf_len = sprintf(uart_buf, "CRC: %X \n", (unsigned int) inc_msg.crc);
    HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100); // what we got
#endif
    return true;

}
