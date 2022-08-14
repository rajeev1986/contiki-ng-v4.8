#include "contiki.h"

#include "nrfx_config.h"
#include "nrfx_twim.h"
#include "hal/nrf_gpio.h"

#define I2C_SDA 26
#define I2C_SCL 27
/******************************************************************************
 * 						Variables
 *****************************************************************************/
static nrfx_twim_t i2c_instance = NRFX_TWIM_INSTANCE(0);
const nrfx_twim_config_t i2c_config =
		{
				.scl 				=	I2C_SCL,
				.sda				=	I2C_SDA,
				.frequency			=	NRF_TWIM_FREQ_100K,
				.interrupt_priority	=	NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
				.hold_bus_uninit	=	false
		};
volatile nrfx_twim_xfer_desc_t i2c_tx_desc;
volatile bool nrf_i2c_transfer_done = false;
volatile bool nrf_i2c_transfer_err = false;

/******************************************************************************
 * 						Functions
 *****************************************************************************/

// void i2c_init(void)
// {
// 	nrfx_err_t error;

// 	nrf_gpio_cfg(i2c_config.sda, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
// 		             NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);

// 	nrf_gpio_cfg(i2c_config.scl, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
// 					 NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);

// 	error = nrfx_twim_init(&i2c_instance, &i2c_config, i2c_event_handler, NULL);
// 	if (error == NRFX_SUCCESS)
// 	{
// 		nrfx_twim_enable(&i2c_instance);
// 	}

// }

// void i2c_write(uint8_t address,uint8_t reg_address,uint8_t *data,uint8_t len)
// {
// 	uint8_t buffer[len + 1];

// 	buffer[0] = reg_address;
// 	memcpy(&buffer[1], data, len);

// 	nrfx_twim_xfer_desc_t write_desc = NRFX_TWIM_XFER_DESC_TX(address,buffer,len+1);
// 	nrfx_err_t error = nrfx_twim_xfer(&i2c_instance, &write_desc, 0);
// 	if (error != NRF_SUCCESS)
// 	// {printf("NRF FAILIURE TX: %ld \n", error);}
// 	i2c_wait();
// }

// void i2c_read(uint8_t address,uint8_t reg_address,uint8_t *data,uint8_t len)
// {
// /*
// 	nrfx_err_t error = nrfx_twim_tx(&i2c_instance,address,&reg_address,1,true);
// 	if (error != NRF_SUCCESS){printf("NRF FAILIURE TX: %ld \n", error);}
// 	error = nrfx_twim_rx(&i2c_instance,address,data,len);
// 	if (error != NRF_SUCCESS){printf("NRF FAILIURE TX: %ld \n", error);} */

// 	nrfx_twim_xfer_desc_t read_desc = NRFX_TWIM_XFER_DESC_TXRX(address,&reg_address,1,data,len);
// 	nrfx_err_t error = nrfx_twim_xfer(&i2c_instance, &read_desc, 0);
// 	if (error != NRF_SUCCESS)
// 	// {printf("NRF FAILIURE TX: %ld \n", error);}
// 	i2c_wait();
// }

// void i2c_event_handler(nrfx_twim_evt_t const * p_event, void * p_context)
// {
// 	switch(p_event->type)
// 		{
// 		case NRFX_TWIM_EVT_DONE: nrf_i2c_transfer_done = true;
// 			//printf("NRFX_TWIM_EVT_DONE \n");
// 			break;
// 		case NRFX_TWIM_EVT_ADDRESS_NACK: nrf_i2c_transfer_err = true;
// 			//printf("NRFX_TWIM_EVT_ADDRESS_NACK \n");
// 			break;
// 		case NRFX_TWIM_EVT_DATA_NACK: nrf_i2c_transfer_err = true;
// 			//printf("NRFX_TWIM_EVT_DATA_NACK \n");
// 			break;
// 		case NRFX_TWIM_EVT_OVERRUN: nrf_i2c_transfer_err = true;
// 			//printf("NRFX_TWIM_EVT_OVERRUN \n");
// 			break;
// 		case NRFX_TWIM_EVT_BUS_ERROR: nrf_i2c_transfer_err = true;
// 			//printf("NRFX_TWIM_EVT_BUS_ERROR \n");
// 			break;
// 		default:
// 			//printf("EVNT HANDLER NONE OF THE ABOVE \n");
// 			break;
// 		}
// }

// void i2c_wait(void)
// {
// 	while(nrf_i2c_transfer_done == false){
// 		if (nrf_i2c_transfer_err == true)
// 		{
// 			printf("I2C TRANSFER ERROR \n");
// 			nrf_i2c_transfer_err = false;
// 			break;
// 		}
// 	}
// 	nrf_i2c_transfer_done = false;
// }

