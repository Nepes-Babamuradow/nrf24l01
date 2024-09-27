/*
 * Nepes Babamuradow. 2.cpp
 *
 * Created: 10.04.2023 11:11:14
 * Author : Admin
 */ 
#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>


//#include <mega32.h>
//#include <delay.h>
   
#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nrf24l01registers.h"
//#include "nrf24l01.h"
#define  nRF_DDR DDRB
#define nRF_PORT PORTB 
#define nRF_PIN PINB
#define  SCK_pin PB5
#define MISO_pin PB4
#define  MOSI_pin PB3
#define  CSN_pin PB2
#define  CE_pin PB1 
#define sbi(pin) nRF_PORT |=(1<<pin)
#define cbi(pin) nRF_PORT &=~(1<<pin)
void SPI_init(void)
{ nRF_DDR=(1<<CE_pin)|(1<SCK_pin)|(1<<CSN_pin)|(1<<MOSI_pin);
	sbi(CSN_pin);
	SPCR=(1<<MSTR) |(1<<SPE);
}
uint8_t SPI_byte(uint8_t byte)
{
	SPDR=byte;
	while(!(SPSR&(1<<SPIF)));
	return SPDR;

}     
void nRF_command (uint8_t command)
{cbi(CSN_pin);
	SPI_byte(command);
	sbi(CSN_pin);
}
void nRF_write_register(uint8_t reg,uint8_t data_out)
{
	reg|=NRF24L01_CMD_W_REGISTER;
	cbi(CSN_pin);
	SPI_byte(reg);
	SPI_byte(data_out);
	sbi(CSN_pin);
}
uint8_t nRF_read_register(uint8_t reg)
{uint8_t data_in;
	cbi(CSN_pin);
	data_in=SPI_byte(reg);
	if(reg!=NRF24L01_REG_STATUS)
	{
		data_in=SPI_byte(NRF24L01_CMD_NOP );
	}
	sbi(CSN_pin);
	return data_in;
	
	}
uint8_t payload , rx_dr_flag;


ISR(INT0_vect)
{
	payload=nRF_read_register (NRF24L01_CMD_R_RX_PAYLOAD);
	rx_dr_flag=1;
	nRF_write_register( NRF24L01_REG_STATUS,0x70);
	}
	
int main(void)
{ SPI_init();
	_delay_ms(100);
	nRF_write_register(NRF24L01_REG_CONFIG,(1<< NRF24L01_REG_MASK_TX_DS)|(1<< NRF24L01_REG_MASK_MAX_RT)|(1<< NRF24L01_REG_EN_CRC)|(1<< NRF24L01_REG_PWR_UP)|(1<< NRF24L01_REG_PRIM_RX));
	_delay_ms(2);
	nRF_write_register( NRF24L01_REG_RX_PW_P0,1);
	sbi(CE_pin);
	_delay_ms(135);
	nRF_write_register(NRF24L01_REG_STATUS,0x70);
	nRF_command (NRF24L01_CMD_FLUSH_TX);
	nRF_command(NRF24L01_CMD_FLUSH_RX);
	MCUCR=(1<<ISC01)|(0<<ISC00);
	GICR=(1<<INT0);
	sei();
	
	
 
while (1)
      { if(rx_dr_flag)
		  {
			  rx_dr_flag=0;
			 
		  }
              
    }
}

