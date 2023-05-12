/***************************************************************************************************************************************************************
 * @file qspi_direct.c
 * @hmi v1 code for EFM32GG11
 * @version x.x.x
 * @provides the HMI comminication protocol between V1 and HMI
 * UART0 is used to communicate between EFM32 board and MCU board through PC4 and PC5 connected to EXP.7 and EXP.9 on EFM32 board respectively. 19200 bps
 * USART5 is used to communicate between EFM32 board and HMI display through PE8 and PE9 connected to EXP.12 and EXP.14 respectively on EFM32 board. 115200 bps
 * @written by Igor Zavgorodni
 ***************************************************************************************************************************************************************/

#include  "em_device.h"
#include  "em_chip.h"
#include  "em_cmu.h"
#include  "em_qspi.h"
#include  "em_gpio.h"
#include  "em_usart.h"
#include  "bsp.h"
#include  "retargetserial.h"
#include  <stdio.h>
#include  <string.h>
#include  <stdlib.h>

void delay(uint32_t dlyTicks);
void clear_uart_buffer(uint8_t uart_number);
void receive_irg_usart5_wait_time(uint16_t wait_time);
uint16_t gen_custom_crc(uint8_t *value, uint16_t size);
void uart0_transmit(uint8_t address, uint8_t length, uint8_t command);
uint8_t* parse_float_ascii_value(uint8_t* start_ptr, char *buffer, uint8_t size);
float bytes_to_float(char *s);
void float_toBytes (float val, char* bytes_array);
void compare_buffers(char* buff0, char* buff1, uint8_t number_counts, uint8_t flag);
void receive_config_state (uint32_t wait);



#define BUFFER_SIZE_UART_0                  512                                                                                                         // receive buffer size
#define BUFFER_SIZE_USART_5                 512                                                                                                         // receive buffer size
#define RS485_INPUT                         GPIO_PinModeSet(gpioPortA, 13, gpioModePushPull, 0)                                                         // RS485 macros RX
#define RS485_OUTPUT                        GPIO_PinModeSet(gpioPortA, 13, gpioModePushPull, 1)                                                         // RS485 macros TX
#define MASTER_ADDRESS                      0x00                                                                                                        // master address MCU 8051
#define SLAVE_ADDRESS                       0xFE                                                                                                        // ESP32 HMI board address
#define SHORT_DELAY                         60000                                                                                                       // 1 minute delay
#define LONG_DELAY                          100000                                                                                                      // 1.40 sec delay
#define BUSY_STATE                          state_register & 0x0001                                                                                     // busy state macros
#define GEN_ON_STATUS                       status_register & 0x0001                                                                                    // generator is running macros
#define GEN_OFF_STATUS                      !(status_register & 0x0001)                                                                                 // generator is off macros

uint8_t                                     data_buffer_uart_0[BUFFER_SIZE_UART_0];                                                                     // uart0_irq_buffer
uint8_t                                     *ptrDataBufferUart0;                                                                                        // pointer to uart0_irq buffer
uint8_t                                     data_buffer_uart_5[BUFFER_SIZE_USART_5];                                                                    // uart5_irq_buffer
uint8_t                                     *ptrDataBufferUsart5;                                                                                       // pointer yo uart5_irq buffer
uint8_t                                     uart0_config_buffer[512];                                                                                   // buffer to sent with updated configuration
uint8_t                                     *ptrUart0ConfigBuffer = uart0_config_buffer;                                                                // pointer to uart0_config_buffer
uint8_t                                     uart0_config_index = 0;                                                                                     // update configuration index
uint16_t                                    i;
uint32_t                                    buffer_index_uart_0 = 0;                                                                                    // uart0_irq index
uint32_t                                    buffer_index_uart_5 = 0;                                                                                    // uart5_irq index
volatile unsigned int                       data_byte_received_flag_uart0 = 0;                                                                          // uart0 byte received flag
volatile unsigned int                       data_byte_received_flag_usart5 = 0;                                                                         // uart5 byte received flag
uint8_t                                     frame_count = 0;                                                                                            // frame count in uart0_irq routine
uint8_t                                     uart0_byte;                                                                                                 // stores the uart0 byte in irq
uint8_t                                     usart5_byte;                                                                                                // stores the usart5 byte in irq
uint16_t                                    state_register, status_register;                                                                            // stores the status and state of the V1
uint8_t                                     RETRIEVE_MEASUREMENT_COMMAND = 0x01;
uint8_t                                     RETRIEVE_CONFIGURATION_COMMAND = 0x02;
uint8_t                                     UPDATE_CONFIGURATION_COMMAND = 0x03;
uint8_t                                     TOGGLE_GENERATOR_COMMAND = 0x04;
uint16_t                                    request_measurement_length = 0x01;
uint16_t                                    request_configuration_length = 0x01;
uint16_t                                    transmit_configuration_length = 0x99;
uint16_t                                    toggle_generator_length = 0x01;
uint8_t                                     warm_up_run_time_flag = 0;
uint8_t                                     generator_type_flag = 0;
uint8_t                                     relay_k3_delay_time_flag = 0;
uint8_t                                     relay_k3_activation_time_flag = 0;
uint8_t                                     relay_k4_delay_time_flag = 0;
uint8_t                                     relay_k4_activation_time_flag = 0;
uint8_t                                     hour_meter_delay_flag = 0;
uint8_t                                     hour_meter_polarity_flag = 0;
volatile uint32_t                           msTicks;                                                                                                    // counts 1ms timeTicks

/*
 * buffers to store data when measurement data is received and parsed
 */
char rssi_buff[4];
char input_current_buff[4];
char load_current_buff[4];
char slave_input_current_buff[4];
char input_voltage_buff[4];
char output_current_buff[4];
char slave_output_current_buff[4];
char slave_load_current_buff[4];
char output_voltage_buff[4];
char generator_voltage_buff[4];
char temperature_sensor_0_buff[4];
char temperature_sensor_1_buff[4];
char temperature_sensor_2_buff[4];
char temperature_sensor_3_buff[4];
char temperature_sensor_4_buff[4];
char temperature_sensor_slave_buff[4];
char slave_input_voltage_buff[4];

/*
 * buffers to store data when configuration data is received and parsed
 */
char router_name_buff[32];
char router_password_buff[32];
char modem_apn_buff[32];
char serial_number_buff[16];
char version_number_buff[4];
char low_trigger_buff[4];
char high_cutoff_buff[4];
char under_voltage_lockout_buff[4];
char warmup_run_time_buff[4];
char generator_run_time_buff[4];
char generator_cooldown_time_buff[4];
char generator_type_buff[4];
char relay_k3_delay_time_buff[4];
char relay_k3_activation_time_buff[4];
char relay_k4_delay_time_buff[4];
char relay_k4_activation_time_buff[4];
char hour_meter_delay_buff[4];
char hour_meter_polarity_buff[4];
char generator_retry_limit_buff[4];

/*
 * buffers to store data when data is entered from HMI to
 * update configuration
 */
char router_name_buff_display[32];
char router_password_buff_display[32];
char modem_apn_buff_display[32];
char low_trigger_value_buff_display[4];
char high_cutoff_value_buff_display[4];
char under_voltage_lockout_value_buff_display[4];
char warmup_run_time_buff_display[4];
char generator_run_time_buff_display[4];
char generator_cooldown_time_buff_display[4];
char generator_type_buff_display[4];
char relay_k3_delay_time_buff_display[4];
char relay_k3_activation_time_buff_display[4];
char relay_k4_delay_time_buff_display[4];
char relay_k4_activation_time_buff_display[4];
char hour_meter_delay_buff_display[4];
char hour_meter_polarity_buff_display[4];
char generator_retry_limit_buff_display[4];

/*
 * HMI strings to be transmitted to display for all routines
 */
char page_indicator_t5[] = "DIAGNOSTIC1.t5.txt=\"";
char page_indicator_t6[] = "DIAGNOSTIC1.t6.txt=\"";
char page_indicator_t7[] = "DIAGNOSTIC1.t7.txt=\"";
char page_indicator_t8[] = "DIAGNOSTIC1.t8.txt=\"";
char page_indicator_t9[] = "DIAGNOSTIC1.t9.txt=\"";
char page_indicator_t10[] = "DIAGNOSTIC1.t10.txt=\"";
char page_indicator_t11[] = "DIAGNOSTIC1.t11.txt=\"";
char page_indicator_t12[] = "DIAGNOSTIC1.t12.txt=\"";
char page_indicator_t13[] = "DIAGNOSTIC1.t13.txt=\"";
char page_indicator_t14[] = "DIAGNOSTIC1.t14.txt=\"";
char page_indicator_t40[] = "DIAGNOSTIC2.t40.txt=\"";
char page_indicator_t15[] = "DIAGNOSTIC2.t15.txt=\"";
char page_indicator_t16[] = "DIAGNOSTIC2.t16.txt=\"";
char page_indicator_t17[] = "DIAGNOSTIC2.t17.txt=\"";
char page_indicator_t18[] = "DIAGNOSTIC2.t18.txt=\"";
char page_indicator_t19[] = "DIAGNOSTIC2.t19.txt=\"";
char page_indicator_t20[] = "DIAGNOSTIC2.t20.txt=\"";

char page_indicator_router_name[] = "CONFIGURATION1.t22.txt=\"";
char page_indicator_router_password[] = "CONFIGURATION1.t23.txt=\"";
char page_indicator_modem_apn[] = "CONFIGURATION1.t21.txt=\"";
char page_indicator_serial_number[] = "CONFIGURATION1.t41.txt=\"";
char page_indicator_version_number[] = "CONFIGURATION1.t42.txt=\"";
char page_indicator_low_trigger[] = "CONFIGURATION1.t25.txt=\"";
char page_indicator_high_cutoff[] = "CONFIGURATION1.t26.txt=\"";
char page_indicator_under_vlotage_lockout[] = "CONFIGURATION1.t27.txt=\"";
char page_indicator_warmup_runtime[] = "CONFIGURATION1.t28.txt=\"";
char page_indicator_generator_runtime[] = "CONFIGURATION1.t29.txt=\"";
char page_indicator_generator_cooldown_time[] = "CONFIGURATION1.t30.txt=\"";
char page_indicator_generator_type[] = "CONFIGURATION1.t24.txt=\"";
char page_indicator_relay_k3_delay[] = "CONFIGURATION2.t31.txt=\"";
char page_indicator_relay_k3_actiivation[] = "CONFIGURATION2.t32.txt=\"";
char page_indicator_relay_k4_delay[] = "CONFIGURATION2.t33.txt=\"";
char page_indicator_relay_k4_activation[] = "CONFIGURATION2.t34.txt=\"";
char page_indicator_hour_meter_delay[] = "CONFIGURATION2.t35.txt=\"";
char page_indicator_hour_meter_polarity[] = "CONFIGURATION2.t36.txt=\"";
char page_indicator_generator_retry_limit[] = "CONFIGURATION2.t37.txt=\"";

char page_indicator_config_received_c1[] = "CONFIGURATION1.t31.txt=\"CONFIGURATION RECEIVED";
char page_indicator_config_received_c2[] = "CONFIGURATION2.t13.txt=\"CONFIGURATION RECEIVED";
char page_indicator_config_not_full_c1[] = "CONFIGURATION1.t31.txt=\"CONFIGURATION IS NOT FULL";
char page_indicator_config_not_full_c2[] = "CONFIGURATION2.t13.txt=\"CONFIGURATION IS NOT FULL";
char page_indicator_config_button_c1[] = "CONFIGURATION1.t31.txt=\"PRESS RECEIVE BUTTON AGAIN";
char page_indicator_config_button_c2[] = "CONFIGURATION2.t13.txt=\"PRESS RECEIVE BUTTON AGAIN";
char page_indicator_config_not_received_c1[] = "CONFIGURATION1.t31.txt=\"CONFIGURATION IS NOT RECEIVED.TRY AGAIN...";
char page_indicator_config_not_received_c2[] = "CONFIGURATION2.t13.txt=\"CONFIGURATION IS NOT RECEIVED.TRY AGAIN...";
char page_indicator_config_please_wait_c1[] = "CONFIGURATION1.t31.txt=\"PLEASE WAIT...";
char page_indicator_config_please_wait_c2[] = "CONFIGURATION2.t13.txt=\"PLEASE WAIT...";
char page_indicator_clear_config1[] = "CONFIGURATION1.t31.txt=\"";
char page_indicator_clear_config2[] = "CONFIGURATION2.t13.txt=\"";
char page_indicator_config_updating_c1[] = "CONFIGURATION1.t31.txt=\"CONFIGURATION IS UPDATING. PLEASE WAIT...";
char page_indicator_config_updating_c2[] = "CONFIGURATION2.t13.txt=\"CONFIGURATION IS UPDATING. PLEASE WAIT...";
char page_indicator_config_updating_values_c1[] = "CONFIGURATION1.t31.txt=\"VALUES ARE INCORRECT OR HASN'T BEEN CHANGED. TRY AGAIN...";
char page_indicator_config_updating_values_c2[] = "CONFIGURATION2.t13.txt=\"VALUES ARE INCORRECT OR HASN'T BEEN CHANGED. TRY AGAIN...";
char page_indicator_generator_startup_c1[] = "CONFIGURATION1.t31.txt=\"GENERATOR STARTUP";
char page_indicator_generator_startup_c2[] = "CONFIGURATION2.t13.txt=\"GENERATOR STARTUP";
char page_indicator_failed_startup_c1[] = "CONFIGURATION1.t31.txt=\"FAILED TO START GENERATOR, TRY AGAIN";
char page_indicator_failed_startup_c2[] = "CONFIGURATION2.t13.txt=\"FAILED TO START GENERATOR, TRY AGAIN";
char page_indicator_generator_on_c1[] = "CONFIGURATION1.t31.txt=\"GENERATOR IS ON, CAN'T TURN ON OR BUSY";
char page_indicator_generator_on_c2[] = "CONFIGURATION2.t13.txt=\"GENERATOR IS ON, CAN'T TURN ON OR BUSY";
char page_indicator_generator_cooldown_c1[] = "CONFIGURATION1.t31.txt=\"GENERATOR COOLDOWN";
char page_indicator_generator_cooldown_c2[] = "CONFIGURATION2.t13.txt=\"GENERATOR COOLDOWN";
char page_indicator_failed_stop_c1[] = "CONFIGURATION1.t31.txt=\"FAILED TO STOP GENERATOR, TRY AGAIN";
char page_indicator_failed_stop_c2[] = "CONFIGURATION2.t13.txt=\"FAILED TO STOP GENERATOR, TRY AGAIN";
char page_indicator_generator_off_c1[] = "CONFIGURATION1.t31.txt=\"GENERATOR IS OFF, CAN'T TURN OFF OR BUSY";
char page_indicator_generator_off_c2[] = "CONFIGURATION2.t13.txt=\"GENERATOR IS OFF, CAN'T TURN OFF OR BUSY";

char page_indicator_unable_to_initiate_server[] = "DIAGNOSTIC1.t2.txt=\"UNABLE TO INITIATE SERVER";
char page_indicator_unable_config_updating_server_rotary[] = "DIAGNOSTIC1.t2.txt=\"CONFIGURATION UPDATING BY SERVER_ROTARY";
char page_indicator_calibration_state[] = "DIAGNOSTIC1.t1.txt=\"CALIBRATION STATE";
char page_indicator_configuration_state[] = "DIAGNOSTIC1.t1.txt=\"CONFIGURATION STATE";
char page_indicator_error_state[] = "DIAGNOSTIC1.t1.txt=\"ERROR STATE";
char page_indicator_emergency_shutdown_state[] = "DIAGNOSTIC1.t1.txt=\"EMERGENCY SHUTDOWN STATE";
char page_indicator_gen_cooldown_state[] = "DIAGNOSTIC1.t1.txt=\"GENERATOR_COOLDOWN_STATE";
char page_indicator_bulk_charging_state[] = "DIAGNOSTIC1.t1.txt=\"BULK CHARGING STATE";
char page_indicator_gen_startup_state[] = "DIAGNOSTIC1.t1.txt=\"GENERATOR STARTUP STATE";
char page_indicator_gen_warmup_state[] = "DIAGNOSTIC1.t1.txt=\"GENERATOR WARMUP STATE";
char page_indicator_idle_state[] = "DIAGNOSTIC1.t1.txt=\"IDLE STATE";
char page_indicator_init_server_comm_state[] = "DIAGNOSTIC1.t1.txt=\"INIT SERVER COMMUNICATION";
char page_indicator_initialization_state[] = "DIAGNOSTIC1.t1.txt=\"INITIALIZATION STATE";

char clear_page_t2[] = "DIAGNOSTIC1.t2.txt=\"";
char page_indicator_gen_on[] = "DIAGNOSTIC1.t3.txt=\"GENERATOR IS RUNNING";
char page_indicator_gen_off[] = "DIAGNOSTIC1.t3.txt=\"GENERATOR IS OFF";
char page_indicator_warmup[] = "DIAGNOSTIC1.t3.txt=\"WARMUP SYSTEM IS ACTIVE";
char page_indicator_gen_failed_start[] = "DIAGNOSTIC1.t3.txt=\"GENERATOR FAILED TO START";
char page_indicator_gen_stopped_prem[] = "DIAGNOSTIC1.t3.txt=\"GENERATOR STOPPED PREMATURELY";
char page_indicator_gen_failed_stop[] = "DIAGNOSTIC1.t3.txt=\"GENERATOR FAILED TO STOP";
char page_indicator_gen_failed_charge_batt[] = "DIAGNOSTIC1.t3.txt=\"GENERATOR FAILED TO CHARGE BATTERY";
char page_indicator_ac_input_fault[] = "DIAGNOSTIC1.t3.txt=\"AC INPUT FAULT";
char page_indicator_undervoltage_lockout_fault[] = "DIAGNOSTIC1.t3.txt=\"UNDERVOLTAGE LOCKOUT FAULT";
char page_indicator_power_output_fault[] = "DIAGNOSTIC1.t3.txt=\"POWER OUTPUT FAULT";
char page_indicator_emergrncy_switc_pressed[] = "DIAGNOSTIC1.t3.txt=\"EMERGENCY SWITCH PRESSED";
char page_indicator_stuck_switch_fault[] = "DIAGNOSTIC1.t3.txt=\"STUCK SWITCH FAULT";
char page_indicator_smb_fault[] = "DIAGNOSTIC1.t3.txt=\"SMB FAULT";
char page_indicator_slave_charger_runtime_error[] = "DIAGNOSTIC1.t3.txt=\"SLAVE CHARGER RUNTIME ERROR";
char page_indicator_slave_charger_ac_input_error[] = "DIAGNOSTIC1.t3.txt=\"SLAVE_CHARGER AC INPUT ERROR";
char page_indicator_slave_charger_in_charging_state[] = "DIAGNOSTIC1.t3.txt=\"SLAVE CHARGER IS IN CHARGING STATE";

void vcom_init(void)                                                                                                                                    /*USART4 PH4->US4_TX #4; PH5->US4_RX #4;
                                                                                                                                                        PH8->US4_CTS #4;
                                                                                                                                                        PH9->US4_RTS #4; PE1->VCOM_ENABLE*/
{
    CMU_ClockEnable(cmuClock_GPIO,true);                                                                                                                // enable GPIO clk prior to write its registers

    GPIO_PinModeSet(gpioPortE,1,gpioModePushPull,1);                                                                                                    /* enable the switch with VCOM_ENABLE pin,
                                                                                                                                                        see User's Guide of the board*/
    RETARGET_SerialInit();
    RETARGET_SerialCrLf(1);

    //printf("VCOM is working!\n");
}


void uart0_init(void)
{
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

  init.baudrate = 19200;                                                                                                                                // baudrate is changed from 115200 bps to 19200 bps

  CMU_ClockEnable(cmuClock_GPIO, true);                                                                                                                 // enable oscillator for GPIO
  CMU_ClockEnable(cmuClock_UART0, true);                                                                                                                // enable oscillator for UART0

  GPIO_PinModeSet(gpioPortC, 5, gpioModeInput, 0);                                                                                                      // pin mode is input UART0_RX -> EXP.9
  GPIO_PinModeSet(gpioPortC, 4, gpioModePushPull, 1);                                                                                                   // pin mode is push/pull UART0_TX->EXP.7

  USART_InitAsync(UART0, &init);                                                                                                                        // Initialize UART0 asynchronous

  NVIC_ClearPendingIRQ(UART0_RX_IRQn);                                                                                                                  // clear the interrupt flag before enabling
  NVIC_EnableIRQ(UART0_RX_IRQn);                                                                                                                        // enable interrupt on UART0_RX

  // routing configuration based on pin alternative functionality
  UART0->ROUTELOC0 = UART_ROUTELOC0_RXLOC_LOC4 | UART_ROUTELOC0_TXLOC_LOC4;                                                                             // EXP.7 -> UART0_TX and EXP.9-> UART0_RX
  UART0->ROUTEPEN |= UART_ROUTEPEN_TXPEN | UART_ROUTEPEN_RXPEN;                                                                                         // enable alternating functionality

  // Enable receive data valid interrupt
  USART_IntEnable(UART0, UART_IEN_RXDATAV);                                                                                                             // enable UART0
}

void usart5_init(void)
{
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

  CMU_ClockEnable(cmuClock_GPIO, true);                                                                                                                 // enable oscillator for GPIO
  CMU_ClockEnable(cmuClock_USART5, true);                                                                                                               // enable oscillator for USART5

  GPIO_PinModeSet(gpioPortE, 9, gpioModeInput, 0);                                                                                                      // pin mode is input USART5_RX -> EXP.14
  GPIO_PinModeSet(gpioPortE, 8, gpioModePushPull, 1);                                                                                                   // pin mode is push/pull USART5_TX -> EXP.12

  USART_InitAsync(USART5, &init);                                                                                                                       // Initialize USART5 asynchronous

  // Enable NVIC USART sources
  NVIC_ClearPendingIRQ(USART5_RX_IRQn);                                                                                                                 // clear the interrupt flag before enabling
  NVIC_EnableIRQ(USART5_RX_IRQn);                                                                                                                       // enable interrupt on USART5_RX

  // routing configuration based on pin alternative functionality
  USART5->ROUTELOC0 = USART_ROUTELOC0_RXLOC_LOC0 | USART_ROUTELOC0_TXLOC_LOC0;                                                                          // EXP.12 -> USART5_TX and EXP.14 -> USART5_RX
  USART5->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;                                                                                      // enable alternating functionality

  // Enable receive data valid interrupt
  USART_IntEnable(USART5, USART_IEN_RXDATAV);                                                                                                           // enable USART5
}

void SysTick_Handler(void)
{
    msTicks++;                                                                                                                                          // increment counter necessary in Delay()
}

void delay(uint32_t dlyTicks)
{
    uint32_t curTicks;

    curTicks = msTicks;                                                                                                                                 // value of msTicks at the start of the delay
    while ((msTicks - curTicks) < dlyTicks) ;                                                                                                           // loops until dlyTicks have elapsed
}

void receive_irq_uart0(void)
{
  uint8_t  *rx_tx_pointer;
  uint16_t  length, crc;

  do{
      data_byte_received_flag_uart0 = 0;                                                                                                                // clear the byte receive flag from usart5_IRQ
      delay(10);                                                                                                                                        // delay 10 ms
    } while(data_byte_received_flag_uart0 == 1);                                                                                                        // wait until all data received checking every 10ms
                                                                                                                                                        // if receive_irq_buffer empty or has some data
  if( buffer_index_uart_0 < 5)                                                                                                                          // addr length and crc must always be received
      {clear_uart_buffer(0); return;}                                                                                                                   // clear buffer and re-enable interrupts if received

  rx_tx_pointer = &data_buffer_uart_0[0];                                                                                                               // set pointer to the start of the buffer
  crc = gen_custom_crc(rx_tx_pointer,(buffer_index_uart_0 - 2));                                                                                        // calculate the crc on data received
  if( (crc >> 8) != data_buffer_uart_0[buffer_index_uart_0 - 2])                                                                                        // assign the second byte of the frame to crc byte one
      {clear_uart_buffer(0); return;}                                                                                                                   // return if the calculated CRC does not match that received
  if( (uint8_t) crc != data_buffer_uart_0[buffer_index_uart_0 - 1])                                                                                     // assign the first byte of the frame to crc second one
      {clear_uart_buffer(0); return;}                                                                                                                   // return if the calculated CRC does not match that received
  if( data_buffer_uart_0[0] != MASTER_ADDRESS)                                                                                                          // if first byte doesn't equal 0x00 -> MASTER_ADDRESS
      {clear_uart_buffer(0); return;}                                                                                                                   // return if packet not transmitting to the master address
  length = data_buffer_uart_0[1];                                                                                                                       // second byte of the frame is length byte #1
  length <<= 8; length &= 0xFF00;                                                                                                                       // shift to the left because length in uint16_t
  length |= data_buffer_uart_0[2];                                                                                                                      // first byte of the frame is length byte #2
  if(length == 4)                                                                                                                                       // only process the poll frame for state/status registers
    {
      state_register = data_buffer_uart_0[3];                                                                                                           // byte number three is assigned to the state register
      state_register <<= 8; state_register &= 0xFF00;                                                                                                   // clear the last byte
      state_register |= data_buffer_uart_0[4];                                                                                                          // assign byte number 4 to state_register
      status_register = data_buffer_uart_0[5];                                                                                                          // byte number five is assigned to the status register
      status_register <<= 8; status_register &= 0xFF00;                                                                                                 // clear the last byte of status_register
      status_register |= data_buffer_uart_0[6];                                                                                                         // assign byte number 6 to state_register
      buffer_index_uart_0 = 0;                                                                                                                          // clear the index number
      return;
    }
}

void receive_irg_uart0_wait_time(uint16_t wait_time)                                                                                                    // value of miliseconds to be entered
{
  for( i = 0; i < wait_time; i++)
  {
      if(data_byte_received_flag_uart0)                                                                                                                 // data has been received
      {
          receive_irq_uart0();                                                                                                                          // wait until all data received
          break;                                                                                                                                        // break the for loop if 10 milisends elapsed
      }
      delay(1);                                                                                                                                         /* delay 1 ms x wait_time gives entire waiting time
                                                                                                                                                        until first byte is received */
  }
  return;
}

void uart0_transmit(uint8_t address, uint8_t length, uint8_t command)
{
  uint8_t  *rx_tx_pointer;
  uint16_t  crc, x;

  RS485_OUTPUT;                                                                                                                                         // poll the line HIGH to transmit data
  clear_uart_buffer(0);
  data_buffer_uart_0[buffer_index_uart_0++] = address;                                                                                                  // address = 0x00
  data_buffer_uart_0[buffer_index_uart_0++] = length >> 8;
  data_buffer_uart_0[buffer_index_uart_0++] = length;                                                                                                   // length = 0x0001
  data_buffer_uart_0[buffer_index_uart_0++] = command;                                                                                                  // command = 0x0001
  rx_tx_pointer = &data_buffer_uart_0[0];                                                                                                               // set pointer to the start of the buffer
  crc = gen_custom_crc(rx_tx_pointer, buffer_index_uart_0);                                                                                             // calculate crc based on the previous bytes
  data_buffer_uart_0[buffer_index_uart_0++] = crc >> 8;                                                                                                 // add crc byte #1
  data_buffer_uart_0[buffer_index_uart_0++] = crc;                                                                                                      // add crc byte #2
  for(x = 0; x < buffer_index_uart_0; x++ )
    {
      USART_Tx(UART0, data_buffer_uart_0[x]);                                                                                                           // send buffer_index_uart_0 bytes to HMI
    }
  while(!(UART0->STATUS & USART_STATUS_TXC));                                                                                                           // TX Complete
  clear_uart_buffer(0);                                                                                                                                 // clear the buffer uart0
  RS485_INPUT;                                                                                                                                          // poll the line LOW in receive mode
}

// transmit data_crc
void uart0_transmit_config(uint8_t address, uint8_t length, uint8_t command)
{
  uint16_t  crc, x;

  RS485_OUTPUT;                                                                                                                                         // poll the line HIGH to transmit data
  clear_uart_buffer(0);                                                                                                                                 // clear the uart0_irq_buffer
  uart0_config_buffer[uart0_config_index++] = address;                                                                                                  // add address of MASTER to be transmitted
  uart0_config_buffer[uart0_config_index++] = length >> 8;                                                                                              // add the length byte#1
  uart0_config_buffer[uart0_config_index++] = length;                                                                                                   // add the length byte#2
  uart0_config_buffer[uart0_config_index++] = command;                                                                                                  // add update configuration command
  compare_buffers(router_name_buff, router_name_buff_display, 32, 0);                                                                                   // add 32 bytes of rssi_buff and so on below
  compare_buffers(router_password_buff, router_password_buff_display, 32, 0);
  compare_buffers(modem_apn_buff, modem_apn_buff_display, 32, 0);
  compare_buffers(low_trigger_buff, low_trigger_value_buff_display, 4, 0);
  compare_buffers(high_cutoff_buff, high_cutoff_value_buff_display, 4, 0);
  compare_buffers(under_voltage_lockout_buff, under_voltage_lockout_value_buff_display, 4, 0);
  compare_buffers(warmup_run_time_buff, warmup_run_time_buff_display, 4, warm_up_run_time_flag);
  compare_buffers(generator_run_time_buff, generator_run_time_buff_display, 4, 0);
  compare_buffers(generator_cooldown_time_buff, generator_cooldown_time_buff_display, 4, 0);
  compare_buffers(generator_type_buff, generator_type_buff_display, 4, generator_type_flag);
  compare_buffers(relay_k3_delay_time_buff, relay_k3_delay_time_buff_display, 4, relay_k3_delay_time_flag);
  compare_buffers(relay_k3_activation_time_buff, relay_k3_activation_time_buff_display, 4, relay_k3_activation_time_flag);
  compare_buffers(relay_k4_delay_time_buff, relay_k4_delay_time_buff_display, 4, relay_k4_delay_time_flag);
  compare_buffers(relay_k4_activation_time_buff, relay_k4_activation_time_buff_display, 4, relay_k4_activation_time_flag);
  compare_buffers(hour_meter_delay_buff, hour_meter_delay_buff_display, 4, hour_meter_delay_flag);
  compare_buffers(hour_meter_polarity_buff, hour_meter_polarity_buff_display, 4, hour_meter_polarity_flag);
  compare_buffers(generator_retry_limit_buff, generator_retry_limit_buff_display, 4, 0);
  crc = gen_custom_crc(ptrUart0ConfigBuffer, uart0_config_index);                                                                                       // calculate crc
  uart0_config_buffer[uart0_config_index++] = crc >> 8;                                                                                                 // add the byte #1 to the uart0_config_buffer
  uart0_config_buffer[uart0_config_index++] = crc;                                                                                                      // add the byte #2 to the uart0_config_buffer
  for(x = 0; x < uart0_config_index; x++ )
    {
      USART_Tx(UART0, uart0_config_buffer[x]);                                                                                                          // send all number of bytes to the MASTER
    }
  while(!(UART0->STATUS & USART_STATUS_TXC));                                                                                                           //  wait until all bytes sent
  clear_uart_buffer(0);                                                                                                                                 // clear thr uart0_buffer
  warm_up_run_time_flag = 0;                                                                                                                            // clear warm_up_flag, might be set to 0
  generator_type_flag = 0;                                                                                                                              // clear generator_type_flag, might be set to 0
  relay_k3_delay_time_flag = 0;                                                                                                                         // clear relay_k3_delay_flag, might be set to 0
  relay_k3_activation_time_flag = 0;
  relay_k4_delay_time_flag = 0;
  relay_k4_activation_time_flag = 0;
  hour_meter_delay_flag = 0;
  hour_meter_polarity_flag = 0;
  uart0_config_index = 0;
  RS485_INPUT;
}

/*
 * compare two buffers buffer and display buffer from HMI. If the display buffer is 0 send the buffer
 * received from configuration frame. If the flag set means buffer accept 0 value and it considered
 * as updated buffer.
 */
void compare_buffers(char* buff0, char* buff1, uint8_t number_counts, uint8_t flag)
{
  if (buff1[0] != 0 || flag == 1)
    {
      for(i = 0; i < number_counts; i++ )
        {
          uart0_config_buffer[uart0_config_index++] = buff1[i];                                                                                         // add the number of bytes of display buffer from HMI
        }
    }
  else
    {
      for(i = 0; i < number_counts; i++ )
        {
          uart0_config_buffer[uart0_config_index++] = buff0[i];                                                                                         // add the number of bytes from receive config buffer
        }
    }
  memset(buff1,0,sizeof(buff1));                                                                                                                        // clear the display buffer
}

void receive_irq_usart5(void)
{
  float low_trigger_value;
  float high_cutoff_value;
  float under_voltage_lockout_value;
  float warmup_runtime_value;
  float generator_run_time_value;
  float generator_cooldown_time_value;
  float generator_type_value;
  float relay_k3_delay_time_value;
  float relay_k3_activation_time_value;
  float relay_k4_delay_time_value;
  float relay_k4_activation_time_value;
  float hour_meter_delay_value;
  float hour_meter_polarity_value;
  float generator_retry_limit_value;

  do{
      data_byte_received_flag_usart5 = 0;                                                                                                               // clear the byte receive flag from usart5_IRQ
      delay(10);                                                                                                                                        // delay 10 ms
    } while(data_byte_received_flag_usart5 == 1);                                                                                                       /* wait until all data received checking every 10ms
                                                                                                                                                        if receive_irq_buffer empty or has some data */
  ptrDataBufferUsart5 =   &data_buffer_uart_5[10];                                                                                                      // set pionter to #10 because oh HMI button characteristics
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '2') && (data_buffer_uart_5[9] == '1') )                                              // if button set to 't25'
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, modem_apn_buff_display, 32);                                                   // store 32 bytes in modem_apn_buffer
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '2') && (data_buffer_uart_5[9] == '3') )                                              // if button set to 't23'
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, router_password_buff_display, 32);                                             // store 32 bytes in router_password_buffer
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '2') && (data_buffer_uart_5[9] == '2') )                                              // if button set to 't22'
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, router_name_buff_display, 32);                                                 // store 32 bytes in router_name_buffer
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '2') && (data_buffer_uart_5[9] == '5') )                                              // if button set to 't25'
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, low_trigger_value_buff_display, 4);                                            // store 4 bytes into low_trigger_value_buffer
      low_trigger_value = atof(low_trigger_value_buff_display);                                                                                         // converts string into floating point value
      low_trigger_value = low_trigger_value * 10;                                                                                                       // multiply by 10 based on Glenn document
      float_toBytes(low_trigger_value, &low_trigger_value_buff_display[0]);                                                                             /* store the floating point value into display buffer
                                                                                                                                                        * to be transfred then to HMI. Repeat for all buffers below
                                                                                                                                                        */
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '2') && (data_buffer_uart_5[9] == '6') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, high_cutoff_value_buff_display, 4);
      high_cutoff_value = atof(high_cutoff_value_buff_display);
      high_cutoff_value = high_cutoff_value * 10;
      float_toBytes(high_cutoff_value, &high_cutoff_value_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '2') && (data_buffer_uart_5[9] == '7') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, under_voltage_lockout_value_buff_display, 4);
      under_voltage_lockout_value = atof(under_voltage_lockout_value_buff_display);
      under_voltage_lockout_value = under_voltage_lockout_value * 10;
      float_toBytes(under_voltage_lockout_value, &under_voltage_lockout_value_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '2') && (data_buffer_uart_5[9] == '8') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, warmup_run_time_buff_display, 4);
      warmup_runtime_value = atof(warmup_run_time_buff_display);
      warmup_runtime_value = warmup_runtime_value * 10;
      warm_up_run_time_flag = 1;
      float_toBytes(warmup_runtime_value, &warmup_run_time_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '2') && (data_buffer_uart_5[9] == '9') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, generator_run_time_buff_display, 4);
      generator_run_time_value = atof(generator_run_time_buff_display);
      generator_run_time_value = generator_run_time_value * 10;
      float_toBytes(generator_run_time_value, &generator_run_time_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '3') && (data_buffer_uart_5[9] == '0') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, generator_cooldown_time_buff_display, 4);
      generator_cooldown_time_value = atof(generator_cooldown_time_buff_display);
      generator_cooldown_time_value = generator_cooldown_time_value * 10;
      float_toBytes(generator_cooldown_time_value, &generator_cooldown_time_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '2') && (data_buffer_uart_5[9] == '4') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, generator_type_buff_display, 4);
      generator_type_value = atof(generator_type_buff_display);
      generator_type_value = generator_type_value * 10;
      generator_type_flag = 1;
      float_toBytes(generator_type_value, &generator_type_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '3') && (data_buffer_uart_5[9] == '1') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, relay_k3_delay_time_buff_display, 4);
      relay_k3_delay_time_value = atof(relay_k3_delay_time_buff_display);
      relay_k3_delay_time_flag = 1;
      float_toBytes(relay_k3_delay_time_value, &relay_k3_delay_time_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '3') && (data_buffer_uart_5[9] == '2') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, relay_k3_activation_time_buff_display, 4);
      relay_k3_activation_time_value = atof(relay_k3_activation_time_buff_display);
      relay_k3_activation_time_flag = 1;
      float_toBytes(relay_k3_activation_time_value, &relay_k3_activation_time_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '3') && (data_buffer_uart_5[9] == '3') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, relay_k4_delay_time_buff_display, 4);
      relay_k4_delay_time_value = atof(relay_k4_delay_time_buff_display);
      relay_k4_delay_time_flag = 1;
      float_toBytes(relay_k4_delay_time_value, &relay_k4_delay_time_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '3') && (data_buffer_uart_5[9] == '4') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, relay_k4_activation_time_buff_display, 4);
      relay_k4_activation_time_value = atof(relay_k4_activation_time_buff_display);
      relay_k4_activation_time_flag = 1;
      float_toBytes(relay_k4_activation_time_value, &relay_k4_activation_time_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '3') && (data_buffer_uart_5[9] == '5') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, hour_meter_delay_buff_display, 4);
      hour_meter_delay_value = atof(hour_meter_delay_buff_display);
      hour_meter_delay_flag = 1;
      float_toBytes(hour_meter_delay_value, &hour_meter_delay_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '3') && (data_buffer_uart_5[9] == '6') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, hour_meter_polarity_buff_display, 4);
      hour_meter_polarity_value = atof(hour_meter_polarity_buff_display);
      hour_meter_polarity_flag = 1;
      float_toBytes(hour_meter_polarity_value, &hour_meter_polarity_buff_display[0]);
    }
  if ((data_buffer_uart_5[7] == 't') && (data_buffer_uart_5[8] == '3') && (data_buffer_uart_5[9] == '7') )
    {
      ptrDataBufferUsart5 = parse_float_ascii_value(ptrDataBufferUsart5, generator_retry_limit_buff_display, 4);
      generator_retry_limit_value = atof(generator_retry_limit_buff_display);
      float_toBytes(generator_retry_limit_value, &generator_retry_limit_buff_display[0]);
    }
  return;
}

void receive_irg_usart5_wait_time(uint16_t wait_time)                                                                                                   // value of miliseconds to be entered
{
  for( i = 0; i < wait_time; i++)
  {
      if(data_byte_received_flag_usart5)                                                                                                                // data has been received
      {
          receive_irq_usart5();                                                                                                                         // wait until all data received
          break;                                                                                                                                        // break the for loop
      }
      delay(1);                                                                                                                                         /* delay 1 ms x wait_time gives entire waiting
                                                                                                                                                        time until first byte is received */
  }
  return;
}


uint8_t* parse_float_ascii_value(uint8_t* start_ptr, char *buffer, uint8_t size)
{
  uint16_t  x;
  uint8_t *check_pointer;

  check_pointer = start_ptr;                                                                                                                            // check_pointer is equal to start_pointer
  memcpy(buffer, check_pointer, size);                                                                                                                  // copy the number of bytes into buffer
  for( x = 0; x < size; x++)
    {*ptrDataBufferUart0++ = *check_pointer++;}                                                                                                         // increase pointer by number of bytes
  return check_pointer;                                                                                                                                 // return the increased pointer
}

void parse_measurement_data(void)
{
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, rssi_buff, 4);                                                                       // store 4 bytes in rssi_buff
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, input_current_buff, 4);                                                              // poineter is incremented by 4 to store input_current_buff
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, load_current_buff, 4);                                                               // poineter is incremented by 4 to store load_current_buff
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, slave_input_current_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, input_voltage_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, output_current_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, slave_output_current_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, slave_load_current_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, output_voltage_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, generator_voltage_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, temperature_sensor_0_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, temperature_sensor_1_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, temperature_sensor_2_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, temperature_sensor_3_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, temperature_sensor_4_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, temperature_sensor_slave_buff,4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, slave_input_voltage_buff, 4);                                                        // last parameter to be stored into slave_input_buffer
}

void parse_configuration_data(void)
{
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, router_name_buff, 32);                                                               // store 32 bytes of config_biuffer in router_name_buff
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, router_password_buff, 32);                                                           // poineter is up by 32 bytes to store router_password_buff
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, modem_apn_buff, 32);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, serial_number_buff, 16);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, version_number_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, low_trigger_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, high_cutoff_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, under_voltage_lockout_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, warmup_run_time_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, generator_run_time_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, generator_cooldown_time_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, generator_type_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, relay_k3_delay_time_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, relay_k3_activation_time_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, relay_k4_delay_time_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, relay_k4_activation_time_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, hour_meter_delay_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, hour_meter_polarity_buff, 4);
  ptrDataBufferUart0 = parse_float_ascii_value(ptrDataBufferUart0, generator_retry_limit_buff, 4);                                                      // store the last 4 bytes in gennerator_retry_limit_buffer
}

void hmi_float_to_string_converter(USART_TypeDef *usart, char *float_buffer,  uint8_t value_size, float divisor, char str[])
{
  uint8_t       i;
  char          float_to_string_buff[10];
  char          end_indicator [] = {0xFF, 0xFF, 0xFF};
  float         float_value;

  float_value = bytes_to_float(float_buffer);                                                                                                           // assign "float_buffer" to float value
  float_value = float_value/divisor;                                                                                                                    // divide by divisor if assighned
  sprintf(float_to_string_buff, "%.2f", float_value);                                                                                                   // convert float value to the string "float_to_string_buff"
  if(float_value != -1.00 && float_value != -100. )                                                                                                     // if the float_value != -1.00 or -100. five digits
    {
      for (i = 0 ; str[i] != 0; i++)
          {
            USART_Tx(usart, str[i]);                                                                                                                    // send the string value of the HMI page
          }
      while(!(usart->STATUS & USART_STATUS_TXC));                                                                                                       // TX Complete
      for (i = 0; i < value_size ; i++)
        {
          USART_Tx(usart, (float_to_string_buff[i]));                                                                                                   // send the float value to HMI with regards of valur_size
        }
      while(!(usart->STATUS & USART_STATUS_TXC));                                                                                                       // TX Complete
      USART_Tx(usart, '\"');                                                                                                                            // the byte means end of transmission
      for (i = 0; i < 3 ; i++)
        {
          USART_Tx(usart, (end_indicator[i]));                                                                                                          // the the last three bytes to HMI display
        }
      while(!(usart->STATUS & USART_STATUS_TXC));                                                                                                       // TX Complete
    }
}

void hmi_string_transmission(USART_TypeDef *usart, char *buffer, char str[], uint8_t size)
{
  uint8_t       i;
  char          end_indicator [] = {0xFF, 0xFF, 0xFF};

  for (i = 0 ; str[i] != 0; i++)
    {
      USART_Tx(usart, str[i]);                                                                                                                          // send the string value of the HMI page
    }
  while(!(usart->STATUS & USART_STATUS_TXC));                                                                                                           // TX Complete
  for (i = 0; i < size; i++)
    {
      USART_Tx(usart, (buffer[i]));                                                                                                                     // send the string buffer with specific number of "size"
    }
  while(!(usart->STATUS & USART_STATUS_TXC));                                                                                                           // TX Complete
  USART_Tx(usart, '\"');                                                                                                                                // the byte means end of transmission
  for (i = 0; i < 3 ; i++)
    {
      USART_Tx(usart, (end_indicator[i]));                                                                                                              // the the last three bytes to HMI display
    }
  while(!(usart->STATUS & USART_STATUS_TXC));                                                                                                           // TX Complete
}

void hmi_measurement_dislpaly (void)
{
  hmi_float_to_string_converter(USART5, rssi_buff, 5, 1.01, page_indicator_t5);                                                                         // sends the value of rssi_buff to HMI display
  hmi_float_to_string_converter(USART5, input_current_buff, 5, 1, page_indicator_t6);                                                                   // sends the value of input_current_buff to HMI display
  hmi_float_to_string_converter(USART5, load_current_buff , 5, 1, page_indicator_t7);
  if(state_register & 0x0010)
    {
      hmi_float_to_string_converter(USART5, slave_input_current_buff, 5, 1, page_indicator_t8);
    }
  hmi_float_to_string_converter(USART5, input_voltage_buff, 5, 1, page_indicator_t9);
  hmi_float_to_string_converter(USART5, output_current_buff, 5, 1, page_indicator_t10);
  if(state_register & 0x0010)
    {
      hmi_float_to_string_converter(USART5, slave_output_current_buff, 5, 1, page_indicator_t11);
    }
  hmi_float_to_string_converter(USART5, slave_load_current_buff , 5, 1, page_indicator_t12);
  hmi_float_to_string_converter(USART5, output_voltage_buff, 5, 1, page_indicator_t13);
  hmi_float_to_string_converter(USART5, generator_voltage_buff, 5, 1, page_indicator_t14);
  hmi_float_to_string_converter(USART5, temperature_sensor_0_buff, 5, 10, page_indicator_t15);
  hmi_float_to_string_converter(USART5, temperature_sensor_1_buff, 5, 10,  page_indicator_t16);
  hmi_float_to_string_converter(USART5, temperature_sensor_2_buff, 5, 10,  page_indicator_t17);
  hmi_float_to_string_converter(USART5, temperature_sensor_3_buff, 5, 10,  page_indicator_t18);
  hmi_float_to_string_converter(USART5, temperature_sensor_4_buff, 5, 10,  page_indicator_t19);
  if(state_register & 0x0010)
    {
      hmi_float_to_string_converter(USART5, temperature_sensor_slave_buff, 5, 10,  page_indicator_t20);
    }
  if(state_register & 0x0010)
    {
      hmi_float_to_string_converter(USART5, slave_input_voltage_buff, 5, 10,  page_indicator_t40);                                                    // the bit 4 is set means the slave charger is attached
    }                                                                                                                                                 // sends the value of slave_input_voltage_buff
}

void state_status_registers_display (void)
{
  if(state_register & 0x0008)                                                                                                                         // STATE_REGISTER
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_unable_config_updating_server_rotary);                                           // if bit 3 is set means rotary switch is changeng config
    }
  else
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, clear_page_t2);                                                                                 // othervise rotary switch is not applied
    }
  if(state_register & 0x0004)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_unable_to_initiate_server);                                                      // if bit 2 is set means server is not connected
    }
  else
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, clear_page_t2);                                                                                 // othervise server is connected
    }
  if(state_register & 0x0020)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_calibration_state);                                                              // bit 5 is set means calibration state
    }
  if(state_register & 0x0040)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_configuration_state);                                                            // bit 6 is set means configuration state
    }
  if(state_register & 0x0080)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_error_state);                                                                    // bit 7 is set means error state
    }
  if(state_register & 0x0100)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_emergency_shutdown_state);                                                       // bit 8 is set means emergency shitdown state
    }
  if(state_register & 0x0200)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_gen_cooldown_state);                                                             // bit 9 is set means generator cooldown state
    }
  if(state_register & 0x0400)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_bulk_charging_state);                                                            // bit 10 is set means bulk charging state
    }
  if(state_register & 0x0800)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_gen_startup_state);                                                              // bit 11 is set means generator startup state
    }
  if(state_register & 0x1000)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_gen_warmup_state);                                                               // bit 12 is set means generator warmup state
    }
  if(state_register & 0x2000)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_idle_state);                                                                     // bit 13 is set means idle state
    }
  if(state_register & 0x4000)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_init_server_comm_state);                                                         // bit 14 is set means init server communication state
    }
  if(state_register & 0x8000)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_initialization_state);                                                           // bit 15 is set means initialization state
    }

  if(status_register & 0x0001)                                                                                                                        // STATUS_REGISTER
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_gen_on);                                                                         // bit 0 is set means generator is running status
    }
  else
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_gen_off);                                                                        // othervise generator is off
    }
  if(status_register & 0x0002)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_warmup);                                                                         // bit 1 is set means generator is in warmup
    }
  if(status_register & 0x0004)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_gen_failed_start);                                                               // bit 2 is set means generator failed to start
    }
  if(status_register & 0x0008)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_gen_stopped_prem);                                                               // bit 3 is set menas generator stopped prematurely
    }
  if(status_register & 0x0010)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_gen_failed_stop);                                                                // bit 4 is set means failed to stop within cooldown time settings
    }
  if(status_register & 0x0020)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_gen_failed_charge_batt);                                                         // bit 5 is set means generator failed to charge batteries within its maximum run time settings
    }
  if(status_register & 0x0040)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_ac_input_fault);                                                                 // bit 6 is set means AC input fault
    }
  if(status_register & 0x0080)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_undervoltage_lockout_fault);                                                     // bit 7 is set means undervoltage lockout fault
    }
  if(status_register & 0x0100)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_power_output_fault);                                                             // bit 8 is set means power output fault
    }
  if(status_register & 0x0200)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_emergrncy_switc_pressed);                                                        // bit 9 is set means emergency shutdown switch pressed
    }
  if(status_register & 0x0400)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_stuck_switch_fault);                                                             // bit 10 is set means stuck switch fault
    }
  if(status_register & 0x0800)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_smb_fault);                                                                      // bit 11 is set means SMB bus fault
    }
  if(status_register & 0x2000)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_slave_charger_runtime_error);                                                    // bit 13 is set means slave charger run time error
    }
  if(status_register & 0x4000)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_slave_charger_ac_input_error);                                                   // bit 14 is set means slave charger AC input error condition
    }
  if(status_register & 0x8000)
    {
      hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_slave_charger_in_charging_state);                                                // bit 15 is set means slave charger is in the charging state
    }
}

void hmi_configuration_dislpaly (void)
{
  hmi_string_transmission(USART5, router_name_buff, page_indicator_router_name, 32);                                                                  // sends the 32 bytes of the string stored in router_name_buffer to HMI display
  hmi_string_transmission(USART5, router_password_buff, page_indicator_router_password, 32);                                                          // sends the 32 bytes of the string stored in router_password_buffer to HMI display
  hmi_string_transmission(USART5, modem_apn_buff, page_indicator_modem_apn, 32);
  hmi_string_transmission(USART5, serial_number_buff, page_indicator_serial_number, 16);
  hmi_string_transmission(USART5, version_number_buff, page_indicator_version_number, 4);                                                             // sends the 4 bytes of the "float_to_string_buffer" to HMI display, divisor is 10
  hmi_float_to_string_converter(USART5, low_trigger_buff, 4, 10, page_indicator_low_trigger);
  hmi_float_to_string_converter(USART5, high_cutoff_buff, 4, 10, page_indicator_high_cutoff);
  hmi_float_to_string_converter(USART5, under_voltage_lockout_buff, 4, 10, page_indicator_under_vlotage_lockout);
  hmi_float_to_string_converter(USART5, warmup_run_time_buff, 4, 10, page_indicator_warmup_runtime);
  hmi_float_to_string_converter(USART5, generator_run_time_buff, 4, 10, page_indicator_generator_runtime);
  hmi_float_to_string_converter(USART5, generator_cooldown_time_buff, 4, 10, page_indicator_generator_cooldown_time);
  hmi_float_to_string_converter(USART5, generator_type_buff, 4, 1, page_indicator_generator_type);
  hmi_float_to_string_converter(USART5, relay_k3_delay_time_buff, 4, 1, page_indicator_relay_k3_delay);
  hmi_float_to_string_converter(USART5, relay_k3_activation_time_buff, 4, 1, page_indicator_relay_k3_actiivation);
  hmi_float_to_string_converter(USART5, relay_k4_delay_time_buff, 4, 1, page_indicator_relay_k4_delay);
  hmi_float_to_string_converter(USART5, relay_k4_activation_time_buff, 4, 1, page_indicator_relay_k4_activation);
  hmi_float_to_string_converter(USART5, hour_meter_delay_buff, 4, 1, page_indicator_hour_meter_delay);
  hmi_float_to_string_converter(USART5, hour_meter_polarity_buff, 4, 1, page_indicator_hour_meter_polarity);
  hmi_float_to_string_converter(USART5, generator_retry_limit_buff, 4, 1, page_indicator_generator_retry_limit);

  hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_received_c1);                                                                 // send a string to CONFIG1 page that "CONFIGURATION RECEIVED"
  hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_received_c2);                                                                 // send a string to CONFIG2 page that "CONFIGURATION RECEIVED"
}

/*This function saves the buffer contain inside the union
//with the respective float representaion of 4 x bytes of the buffer
and then return float_Value back to the main loop*/
float bytes_to_float(char *s)
{
  union
  {
  char temp_array[4];
  float float_variable;
  }float_Value;
  float_Value.temp_array[3] = s[0];
  float_Value.temp_array[2] = s[1];
  float_Value.temp_array[1] = s[2];
  float_Value.temp_array[0] = s[3];
  return float_Value.float_variable;
}

//This function puts float number to the floating point buffer
//the float buffer is rreturned
void float_toBytes (float val, char* bytes_array)
{
  union
  {
  float float_variable;
  char temp_array [4];
  }floatValue;
floatValue.float_variable = val;
bytes_array[0] = floatValue.temp_array[3];
bytes_array[1] = floatValue.temp_array[2];
bytes_array[2] = floatValue.temp_array[1];
bytes_array[3] = floatValue.temp_array[0];
}

void clear_uart_buffer(uint8_t uart_number)
{
  uint16_t  x;
  if(uart_number == 0)
    {
      for( x = 0; x < BUFFER_SIZE_UART_0; x++)
      {data_buffer_uart_0[x] = 0;}                                                                                                                    // clear the uart0 data buffer if
                                                                                                                                                      // it exceeds the BUFFER_SIZE
      buffer_index_uart_0 = 0;                                                                                                                        // zero index
      data_byte_received_flag_uart0 = 0;                                                                                                              // clear the received_byte flag for uart0
    }
  else if(uart_number == 5)
    {
      for( x = 0; x < BUFFER_SIZE_USART_5; x++)
      {data_buffer_uart_5[x] = 0;}                                                                                                                    // clear the usart 5 data buffer if
                                                                                                                                                      // it exceeds the BUFFER_SIZE
      buffer_index_uart_5 = 0;                                                                                                                        // zero index
      data_byte_received_flag_usart5 = 0;                                                                                                             // clear the received_byte flag for usart5
    }
  return;
}

void receive_config_state (uint32_t wait)
{
  frame_count = 0;                                                                                                                                    // clear the frame_count
  for(i = 0; i < wait; i++)
    {
     if(data_byte_received_flag_uart0)                                                                                                                // if data received on uart0
       {
         receive_irq_uart0();                                                                                                                         // receive the bytes
         //clear_uart_buffer(5);
         frame_count++;                                                                                                                               // increment the frame_count as we are only interested in poll frame
         if(!(BUSY_STATE) && (data_buffer_uart_0[2] == 0x04))                                                                                         // if not busy poll frame is received
           {
             uart0_transmit(SLAVE_ADDRESS, request_configuration_length, RETRIEVE_CONFIGURATION_COMMAND);                                             // transmit the retrieve configuration command
             receive_irg_uart0_wait_time(750);                                                                                                        // recieve the bytes in response, wait for 750 ms
             if((data_buffer_uart_0[0] == 0x00) && (data_buffer_uart_0[2] == 0xAC))                                                                   // if the received frame is correct
               {
                 ptrDataBufferUart0 = &data_buffer_uart_0[3];                                                                                         // parse it starting from byte 3 to bypass address and frame length
                 parse_configuration_data();                                                                                                          // store all the parsed data into global buffers
                 hmi_configuration_dislpaly();                                                                                                        // transmit the global buffers into HMI display
                 break;                                                                                                                               // break the for loop
               }
             else
               {
                 hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_not_full_c1);                                                  // if the received frame is incorrect, send a message CONFIG1
                 hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_not_full_c2);                                                  // if the received frame is incorrect, send a message CONFIG2
                 break;
               }
           }
         else if (frame_count == 6)                                                                                                                   // if the poll frame is not received 6 times
           {
             hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_not_received_c1);                                                  // if busy or poll frame is incorrect send a message CONFIG1
             hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_not_received_c2);                                                  // if busy or poll frame is incorrect send a message CONFIG1
             frame_count = 0;                                                                                                                         // reset the frame_count
             break;                                                                                                                                   // break the loop
           }
       }
     delay(1);                                                                                                                                        // delay 1 ms
    }
}

int main(void)
{
  CHIP_Init();                                                                                                                                        // chip initialization

  vcom_init();                                                                                                                                        // virtual com port initialization

  uart0_init();                                                                                                                                       // init uart0, uart0->PC4->TX->EXP.7;PC5->RX->EXP.9

  usart5_init();                                                                                                                                      // init usart5 usart5->PE8->TX->EXP.12;PE9->RX->EXP.14

  RS485_INPUT;                                                                                                                                        // transceiver set to receive mode

  if(SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))                                                                                          // Setup SysTick Timer for 1 msec interrupts
  {while(1);}

  while(1)
    {
      if(data_byte_received_flag_uart0)                                                                                                               // if data received on uart0
        {
          receive_irq_uart0();                                                                                                                        // if rteceived dat is poll frame
          state_status_registers_display();                                                                                                           // display state and status of the system on HMI display
          if ((state_register & 0x0080) || (state_register & 0x0200) || (state_register & 0x0400) ||
              (state_register & 0x1000) || (state_register & 0x2000))
                {
                  if((data_buffer_uart_0[0] == 0x00) && (data_buffer_uart_0[2] == 0x04))                                                              // if the poll frame is correct and not truncated
                    {
                      if(!(BUSY_STATE))                                                                                                               // if the system is not busy
                        {
                          uart0_transmit(SLAVE_ADDRESS, request_measurement_length, RETRIEVE_MEASUREMENT_COMMAND);                                    // tramsmit the frame with retrieve measurement command
                          receive_irg_uart0_wait_time(750);                                                                                           // wait 750 ms to receive the measurement frame from MASTER
                          if((data_buffer_uart_0[0] == 0x00) && (data_buffer_uart_0[2] == 0x44))                                                      // if the received frame is correct and not truncated
                            {
                              ptrDataBufferUart0 =   &data_buffer_uart_0[3];                                                                          // parse it starting from byte 3 to bypass address and frame length
                              parse_measurement_data();                                                                                               // parse the measurement data into global buffers
                              hmi_measurement_dislpaly();                                                                                             // transmit the global buffers in HMI display
                              hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_clear_config1);                                          // clear the message in CONFIG1
                              hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_clear_config2);                                          // clear the message in CONFIG2
                            }
                        }
                    }
                }
          clear_uart_buffer(0);                                                                                                                       // clear the uart0_irq_buff
          clear_uart_buffer(5);                                                                                                                       // clear the uart5_irq_buff
        }
/*
 * if the command are received from HMI display
 */
      if(data_byte_received_flag_usart5)
          {
            receive_irq_usart5();                                                                                                                     // receive the frame from HMI diplay and store it in uart5_irq_buff
            if((data_buffer_uart_5[1] == 0x02) && (data_buffer_uart_5[2] == 0x23))                                                                    // if "RECEIVE_CONFIG" button is pressed
              {
                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_please_wait_c1);                                                // send a message "PLEASE WAIT" to CONFIG1
                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_please_wait_c2);                                                // send a message "PLEASE WAIT" to CONFIG2
                receive_config_state(SHORT_DELAY);                                                                                                    // call "receive_config_state" function
              }
            if((data_buffer_uart_5[1] == 0x03) && (data_buffer_uart_5[2] == 0x13))                                                                    // if "UPDATE_CONFIG" button is pressed
              {
                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_please_wait_c1);                                                // send a message "PLEASE WAIT" to CONFIG1
                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_please_wait_c2);                                                // send a message "PLEASE WAIT" to CONFIG2
                for(i = 0; i < SHORT_DELAY; i++)
                  {
                    if(data_byte_received_flag_uart0)                                                                                                 // wait until data is received on uart0
                      {
                        receive_irq_uart0();                                                                                                          // receive and store frame form the MASTER
                        frame_count++;                                                                                                                // start increment frame_count
                        if(!(BUSY_STATE) && (data_buffer_uart_0[2] == 0x04))                                                                          // if poll frame is received and not busy
                          {
                            uart0_transmit_config(SLAVE_ADDRESS, transmit_configuration_length, UPDATE_CONFIGURATION_COMMAND);                        // transmit the full frame to update configuration with all wether data enterd form HMI or not
                            receive_irg_uart0_wait_time(500);                                                                                         // receive/store ACK/NACK
                            if((data_buffer_uart_0[0] == 0x00) && (data_buffer_uart_0[3] == 0x04))                                                    // if ACK
                              {
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_updating_c1);                                   // send a message "configuration is updating" CONFIG1
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_updating_c2);                                   // send a message "configuration is updating" CONFIG2
                                receive_config_state(LONG_DELAY);                                                                                     // receive the updated configuration and updated the HMI display
                                break;                                                                                                                // break the foor loop
                              }
                            else
                              {
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_updating_values_c1);                            // if NACK; send the message that values are incorrect CONFIG1
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_updating_values_c2);                            // if NACK; send the message that values are incorrect CONFIG2
                                break;
                              }
                          }
                        else if (frame_count == 6)                                                                                                    // if the number of frame is more than 6 and not required poll frame is received
                          {
                            hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_not_received_c1);                                   // send the message configuration is not received CONGIG1 page
                            hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_not_received_c2);                                   // send the message configuration is not received CONGIG2 page
                            frame_count = 0;                                                                                                          // reset the frame_count
                            break;                                                                                                                    // break the foor loop
                          }
                      }
                    delay(1);                                                                                                                         // delay 1 ms
                  }
              }

            if((data_buffer_uart_5[1] == 0x03) && (data_buffer_uart_5[2] == 0x15))                                                                    // if "START GENERATOR" button is pressed
              {
                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_please_wait_c1);                                                // send a message "PLEASE WAIT" to CONFIG1
                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_please_wait_c2);                                                // send a message "PLEASE WAIT" to CONFIG2
                for(i = 0; i < SHORT_DELAY; i++)
                  {
                    if(data_byte_received_flag_uart0)
                      {
                        receive_irq_uart0();                                                                                                          // poll frame is received
                        frame_count++;                                                                                                                // count frames
                        if(!(BUSY_STATE) && (data_buffer_uart_0[2] == 0x04) && (GEN_OFF_STATUS))                                                      // if system is not busy, poll frame is received and generator is in OFF status
                          {
                            uart0_transmit(SLAVE_ADDRESS, toggle_generator_length, TOGGLE_GENERATOR_COMMAND);                                         // transint the start generator frame
                            receive_irg_uart0_wait_time(500);                                                                                         // wait for ACK/NACK
                            if((data_buffer_uart_0[0] == 0x00) && (data_buffer_uart_0[3] == 0x04))                                                    // if ACK
                              {
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_generator_startup_c1);                                 // send a message "GENERATOR STARTUP" CONFIG1
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_generator_startup_c2);                                 // send a message "GENERATOR STARTUP" CONFIG2
                                break;
                              }
                            else
                              {
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_failed_startup_c1);                                    // if NACK; send the message "GENERATOR FAILED TO START" CONFIG1
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_failed_startup_c2);                                    // if NACK; send the message "GENERATOR FAILED TO START" CONFIG2
                                break;
                              }
                          }
                        else if (frame_count == 6)                                                                                                    // if the number of frame is more than 6 and not required poll frame is received
                          {
                            hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_generator_on_c1);                                          // send the message that GENERATOR IS ON and CAN'T START CONFIG1 page
                            hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_generator_on_c2);                                          // send the message that GENERATOR IS ON and CAN'T START CONGIG2 page
                            frame_count = 0;                                                                                                          // reset the frame count
                            break;                                                                                                                    // brake the foor loop
                          }
                      }
                    delay(1);                                                                                                                         // 1ms delay
                  }
              }
            if((data_buffer_uart_5[1] == 0x03) && (data_buffer_uart_5[2] == 0x16))                                                                    // if "STOP GENERATOR" button is pressed
              {
                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_please_wait_c1);                                                // send a message "PLEASE WAIT" to CONFIG1
                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_config_please_wait_c2);                                                // send a message "PLEASE WAIT" to CONFIG2
                for(i = 0; i < SHORT_DELAY; i++)
                  {
                    if(data_byte_received_flag_uart0)
                      {
                        receive_irq_uart0();                                                                                                          // poll frame is received
                        frame_count++;                                                                                                                // count frames
                        if(!(BUSY_STATE) && (data_buffer_uart_0[2] == 0x04) && (GEN_ON_STATUS))                                                       // if system is not busy, poll frame is received and generator is in ON status
                          {
                            uart0_transmit(SLAVE_ADDRESS, toggle_generator_length, TOGGLE_GENERATOR_COMMAND);                                         // transint the start generator frame
                            receive_irg_uart0_wait_time(500);                                                                                         // wait for ACK/NACK
                            if((data_buffer_uart_0[0] == 0x00) && (data_buffer_uart_0[3] == 0x04))                                                    // if ACK
                              {
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_generator_cooldown_c1);                                // send a message "GENERATOR COOLDOWN" CONFIG1
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_generator_cooldown_c2);                                // send a message "GENERATOR COOLDOWN" CONFIG2
                                break;
                              }
                            else
                              {
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_failed_stop_c1);                                       // if NACK; send the message "GENERATOR FAILED TO STOP" CONFIG1
                                hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_failed_stop_c2);                                       // if NACK; send the message "GENERATOR FAILED TO STOP" CONFIG2
                                break;
                              }
                          }
                        else if (frame_count == 6)                                                                                                    // if the number of frame is more than 6 and not required poll frame is received
                          {
                            hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_generator_off_c1);                                         // send the message that GENERATOR IS OFF and CAN'T STOP CONFIG1 page
                            hmi_float_to_string_converter(USART5, 0 , 0, 1, page_indicator_generator_off_c2);                                         // send the message configuration is not received CONGIG2 page
                            frame_count = 0;                                                                                                          // reset the frame_count
                            break;                                                                                                                    // break the foor loop
                          }
                      }
                    delay(1);                                                                                                                         // 1ms delay
                  }
              }
            frame_count = 0;                                                                                                                          // reset the frame_count
            clear_uart_buffer(0);                                                                                                                     // clear uart0_irq_buff
            clear_uart_buffer(5);                                                                                                                     // clear uart5_irq_buff
          }
    }
}



void UART0_RX_IRQHandler(void)
{
  if (UART0->IF & UART_IF_RXDATAV)                                                                                                                    // check for RX data valid interrupt
  {
      uart0_byte = UART0->RXDATA;                                                                                                                     // copy a byte into uart0_byte char
      data_buffer_uart_0[buffer_index_uart_0++] = uart0_byte;                                                                                         // copy a byte into data_buffer_uart_0
      if(buffer_index_uart_0 == BUFFER_SIZE_UART_0)
      {buffer_index_uart_0 = 0;}                                                                                                                      // wrap index if at end of buffer
      data_byte_received_flag_uart0 = 1;                                                                                                              // set receive byte flag means we received a byte
  }
  USART_IntClear(UART0, USART_IF_RXDATAV);                                                                                                            // clear pending interrupt flags
}

void USART5_RX_IRQHandler(void)
{
  if (USART5->IF & USART_IF_RXDATAV)                                                                                                                  // check for RX data valid interrupt
  {
      usart5_byte = USART5->RXDATA;                                                                                                                   // copy a byte into uart0_byte char
      data_buffer_uart_5[buffer_index_uart_5++] = usart5_byte;                                                                                        // copy a byte into data_buffer_uart_0
      if(buffer_index_uart_5 == BUFFER_SIZE_USART_5)
      {buffer_index_uart_5 = 0;}                                                                                                                      // wrap index if at end of buffer
      data_byte_received_flag_usart5 = 1;                                                                                                             // set receive byte flag means we received a byte
  }
  USART_IntClear(USART5, USART_IF_RXDATAV);                                                                                                           // clear pending interrupt flags
}

// crc polynomial function
uint16_t gen_custom_crc(uint8_t *value, uint16_t size)
{
  #define                POLYNOMIAL                    0xA473
  uint16_t out = 0x1357;                                                                                                                                                                                                                                                                     // seed value
  uint8_t bits_read = 0;
  uint16_t x, y, crc;

  bool bit_flag;
                if( ! value)
                                {return 0;}                                                                                                                                                                                                                                                                                                                                                           // no content

                while( size > 0)
                                {
                                bit_flag = out >> 15;
                                out <<= 1;
                                out |= (*value >> bits_read) & 1;
                                bits_read++;                                                                                                                                                                                                                                                                                                                                      // increment bits read
                                if( bits_read > 7)
                                                {
                                                bits_read = 0;
                                                value++;                                                                                                                                                                                                                                                                                                                                              // increment pointer
                                                size--;                                                                                                                                                                                                                                                                                                                                                    // decrement size
                                                }
                                if( bit_flag)
                                                {out ^= POLYNOMIAL;}
                                }

                for( x = 0; x < 16; ++x)
                                {                                                                                                                                                                                                                                                                                                                                                                                              // push out the high order byte
                    bit_flag = out >> 15;
                    out <<= 1;
                                if( bit_flag)
                                                {out ^= POLYNOMIAL;}
                                }
                crc = 0;
                x = 0x8000;
                y = 0x0001;
    for (; x != 0; x >>= 1, y <<= 1)                                                                                                                                                                                                                                                                   // shift re-allignment
                                {if (x & out) crc |= y;}

    return crc;
}
