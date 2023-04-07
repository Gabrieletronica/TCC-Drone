/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  NRF24L01.c
  Author:     
  Updated:    16th JANUARY 2023

  ***************************************************************************************************************

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


#include "stm32f1xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1

#define NRF24_CE_PORT   GPIOB
#define NRF24_CE_PIN    GPIO_PIN_0

#define NRF24_CSN_PORT   GPIOA
#define NRF24_CSN_PIN    GPIO_PIN_4

#define BUFFER           10


void CS_Select (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

void CS_UnSelect (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}


void CE_Enable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_Disable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}



// Escrever um único byte para o registrador
void nrf24_WriteReg (uint8_t Reg, uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	buf[1] = Data;

	//Coloca o pino CS em 0 para selecionar o dispositivo
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

	//Coloca o pino CS em 1 para liberar o dispositivo
	CS_UnSelect();
}

//Escrever vários bytes para o registrador
void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
//	buf[1] = Data;

	//Coloca o pino CS em 0 para selecionar o dispositivo
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);

	//Coloca o pino CS em 1 para liberar o dispositivo
	CS_UnSelect();
}


uint8_t nrf24_ReadReg (uint8_t Reg)
{
	uint8_t data=0;

	//Coloca o pino CS em 0 para selecionar o dispositivo
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);

	//Coloca o pino CS em 1 para liberar o dispositivo
	CS_UnSelect();

	return data;
}


/* Lê varios bytes de um registrador do dispostivo */
void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size)
{
	//Coloca o pino CS em 0 para selecionar o dispositivo
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	//Coloca o pino CS em 1 para liberar o dispositivo
	CS_UnSelect();
}


// Envia o comando para o NRF
void nrfsendCmd (uint8_t cmd)
{
	//Coloca o pino CS em 0 para selecionar o dispositivo
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	//Coloca o pino CS em 1 para liberar o dispositivo
	CS_UnSelect();
}

// Reseta os valores do NRF
void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		nrf24_WriteReg(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
	nrf24_WriteReg(CONFIG, 0x08);
	nrf24_WriteReg(EN_AA, 0x3F);
	nrf24_WriteReg(EN_RXADDR, 0x03);
	nrf24_WriteReg(SETUP_AW, 0x03);
	nrf24_WriteReg(SETUP_RETR, 0x03);
	nrf24_WriteReg(RF_CH, 0x02);
	nrf24_WriteReg(RF_SETUP, 0x0E);
	nrf24_WriteReg(STATUS, 0x00);
	nrf24_WriteReg(OBSERVE_TX, 0x00);
	nrf24_WriteReg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_WriteReg(RX_ADDR_P2, 0xC3);
	nrf24_WriteReg(RX_ADDR_P3, 0xC4);
	nrf24_WriteReg(RX_ADDR_P4, 0xC5);
	nrf24_WriteReg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
	nrf24_WriteReg(RX_PW_P0, 0);
	nrf24_WriteReg(RX_PW_P1, 0);
	nrf24_WriteReg(RX_PW_P2, 0);
	nrf24_WriteReg(RX_PW_P3, 0);
	nrf24_WriteReg(RX_PW_P4, 0);
	nrf24_WriteReg(RX_PW_P5, 0);
	nrf24_WriteReg(FIFO_STATUS, 0x11);
	nrf24_WriteReg(DYNPD, 0);
	nrf24_WriteReg(FEATURE, 0);
	}
}



// Inicia o NRF
void NRF24_Init (void)
{

	// desabilita o chip antes de configurar o dispositivo
	CE_Disable();
	// reseta tudo
	nrf24_reset (0);

	nrf24_WriteReg(CONFIG, 0);  // é configurado mais na frente

	nrf24_WriteReg(EN_AA, 0x00);  // Habilita auto ACK

	nrf24_WriteReg (EN_RXADDR, 0);  // não habilita datapipe por enquanto

	nrf24_WriteReg (SETUP_AW, 0x03);  // 5 bytes para o endereço TX/RX 

	nrf24_WriteReg (SETUP_RETR, 0);   // sem retransmissão

	nrf24_WriteReg (RF_CH, 0);  // será configurado durante o Tx ou RX

	nrf24_WriteReg (RF_SETUP, 0x26);   // Potência máxima e velocidade em 250Kbps

	// Habilita o chip depois de configurar o dispositivo
	CE_Enable();

}


// Configura no modo Tx

void NRF24_TxMode (uint8_t *Address, uint8_t channel)
{
	// Desabilita o chip antes de configurar o dispositivo
	CE_Disable();

	nrf24_WriteReg (RF_CH, channel);  // Seleciona o canal

	nrf24_WriteRegMulti(TX_ADDR, Address, 5);  // Seleciona o endereço Tx


	// Aciona o dispositivo no modo TX
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1) | (1<<2) | (1<<3);   // Escreve 1 no PWR_UP bit
	config = config & (0xFE);   // Escreve 0 no PRIM_RX, e 1 no PWR_UP, e os outros bits são mantidos
	nrf24_WriteReg (CONFIG, config);

	// Habilita o chip depois de configurar o dispositivo
	CE_Enable();
}


// Trasmite os dados

uint8_t NRF24_Transmit (uint8_t *data)
{
	uint8_t cmdtosend = 0;
	uint16_t cont = 0;
	// Seleciona o dispositivo
	CS_Select();

	// comando payload
	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	// envia o payload
	HAL_SPI_Transmit(NRF24_SPI, data, BUFFER, 1000);

	// Libera o dispositivo
	CS_UnSelect();

	//delay de 77us
	HAL_Delay(1);
//	while(cont < 5570){
//		cont++;
//	}
//	cont = 0;

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	// verifica se o quarto bit do FIFO_STATUS está vazio 
	if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
	{
		cmdtosend = FLUSH_TX;
		nrfsendCmd(cmdtosend);

		// reseta o FIFO_STATUS
		nrf24_reset (FIFO_STATUS);

		return 1;
	}

	return 0;
}

// Configura no Modo Rx
void NRF24_RxMode (uint8_t *Address, uint8_t channel)
{
	// Desabilita o chip antes de configurar o dispositivo
	CE_Disable();

	nrf24_reset (STATUS);

	nrf24_WriteReg (RF_CH, channel);  // Seleciona o canal

	// Seleciona o data pipe 2
	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<2);
	nrf24_WriteReg (EN_RXADDR, en_rxaddr);

	nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);  // Escreve o endereço Pipe1
	nrf24_WriteReg(RX_ADDR_P2, 0xEE);  // Escreve o endereço LSB Pipe2

	nrf24_WriteReg (RX_PW_P2, 10);   // reserva 10 bit de payload for pipe 2


	// Aciona o dispositivo no modo RX
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1) | (1<<0) | (1<<2) | (1<<3);
	nrf24_WriteReg (CONFIG, config);

	// Habilita o chip depois de configurar o dispositivo
	CE_Enable();
}

// verifica se tem dados no pipe
uint8_t isDataAvailable (int pipenum)
{
	uint8_t status = nrf24_ReadReg(STATUS);

	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{

		nrf24_WriteReg(STATUS, (1<<6));

		return 1;
	}

	return 0;
}

// recebe os dados
void NRF24_Receive (uint8_t *data)
{
	uint8_t cmdtosend = 0;

	// Seleciona o dispositivo
	CS_Select();

	// Comando para o payload
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	// Recebe o payload
	HAL_SPI_Receive(NRF24_SPI, data, BUFFER, 1000);

	// Libera o dispositivo
	CS_UnSelect();

	HAL_Delay(1);

	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);
}



// Lê todos os registradores
void NRF24_ReadAll (uint8_t *data)
{
	for (int i=0; i<10; i++)
	{
		*(data+i) = nrf24_ReadReg(i);
	}

	nrf24_ReadReg_Multi(RX_ADDR_P0, (data+10), 5);

	nrf24_ReadReg_Multi(RX_ADDR_P1, (data+15), 5);

	*(data+20) = nrf24_ReadReg(RX_ADDR_P2);
	*(data+21) = nrf24_ReadReg(RX_ADDR_P3);
	*(data+22) = nrf24_ReadReg(RX_ADDR_P4);
	*(data+23) = nrf24_ReadReg(RX_ADDR_P5);

	nrf24_ReadReg_Multi(RX_ADDR_P0, (data+24), 5);

	for (int i=29; i<38; i++)
	{
		*(data+i) = nrf24_ReadReg(i-12);
	}

}
