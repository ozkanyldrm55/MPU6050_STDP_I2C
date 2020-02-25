#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


/*MPU6050 Sensor slave adresi*/
#define Slave_Address 0x68

char str[50];
int i;
uint16_t a,b,c,a1,b1,c1;
uint8_t m,m1;
uint16_t d,d1;

void Usart_Puts(USART_TypeDef* USARTx , volatile char *s)
{
	while(*s) // s karakteri kadar donderir.
	{
		while(!(USARTx -> SR & 0x00000040)); /* CTS LBD TXE TC RXNE IDLE ORE NF FE PE
		 	 	 	 	 	 	 	 	 	 	 0	 0   0   1   0   0    0  0  0  0	*/	/* TC : aktarim tamamlandiginda 1 olur , boylece while ici 0 olur ve donguden cikar,
												  	  	  	  	  	  	  	  	  	  	  	 * aksi halde while icerisinde bekler aslinda usart modulu musait olana kadar beklememizi saglar*/
		USART_SendData(USARTx,*s);
		*s++;
	}
}

long map(long x , long in_min , long in_max , long out_min , long out_max) /* map = (Degisken adi , donusturulecek ver min , max , donusturulmus veri min ,max) */
{
	return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

void I2C1_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	I2C_InitTypeDef 	I2C_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_APB1Periph_I2C1 , ENABLE); 	// I2C1 icin APB1 clock hatti aktif edildi.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);	// SCL and SDA  icin AHB1 Clock hatti aktif edildi.

	GPIO_PinAFConfig(GPIOB,GPIO_Pin_6,GPIO_AF_I2C1); 		// SCL
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_I2C1);  	// SDA

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 	// PB6 = SCL And PB7 = SDA olarak kullaniyoruz
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 				// pin bacagi GND'ye baglanmis sayilir , Bu durumda pini high olarak cikip vermek için diþaridan pull-up dirençleri eklemeniz gerekmekte.
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 				// Giriþ olarak ayarlanan pinden herhangi bir sinyal gelmediginde, giriþi High seviyesinde tutar.
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	// I2C Konfigutasyonu

	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable; 			// okuma yapacagimiz icin onay cevabini kapattik.
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;// ACK adresi 7 bit olarak sectik
	I2C_InitStruct.I2C_ClockSpeed = 100000; 			// 100 KHz
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;		// 50% duty cycle --> standard
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;				// I2C Modu
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;				// master modda cihazin kendi adresini belirtir
	I2C_Init(I2C1,&I2C_InitStruct);
	I2C_Cmd(I2C1,ENABLE);
}

	void I2C_start(I2C_TypeDef* I2Cx , uint8_t address , uint8_t direction) // direction : okuma yada yazma oldugunu belirtmek icin kullanilmaktadir.
	{
		// i2c bayragi uygun olana kadar bekle.belki baska veri transferi yapmaktadir.
		while(I2C_GetFlagStatus(I2Cx , I2C_FLAG_BUSY));
		// Send I2C1 START condition , I2C1 baslatiyoruz.
		I2C_GenerateSTART(I2Cx , ENABLE);
		// master mod un secilmesini kontrol ediyoruz.master oldugu anda 1 olacak ve tersi olup while den cikacak
		while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT));
		 // hangi slave cihazla iletisime baslayacagimizi seciyoruz(address)
		I2C_Send7bitAddress(I2Cx , address , direction);

		if(direction == I2C_Direction_Transmitter){
			// gonderici olarak hazir hale geldim mi diye sorguluyorum.
		while(!I2C_CheckEvent(I2Cx ,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
			}
		else if(direction == I2C_Direction_Receiver){
			// alici olarak hazir hale geldim mi diye sorguluyorum.
		while(!I2C_CheckEvent(I2Cx , I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
			}
	}


void I2C_write(I2C_TypeDef* I2Cx , uint8_t data)
{
	I2C_SendData(I2Cx , data);
	while(I2C_CheckEvent(I2Cx , I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // gonderme islemi bitene kadar beliyor.
}

uint8_t I2C_Read_Ack(I2C_TypeDef* I2Cx)
{
	I2C_AcknowledgeConfig(I2Cx , ENABLE);
	while(I2C_CheckEvent(I2Cx , I2C_EVENT_MASTER_BYTE_RECEIVED));
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
	I2C_AcknowledgeConfig(I2Cx , DISABLE);
	I2C_GenerateSTOP(I2Cx , ENABLE);
	while(I2C_CheckEvent(I2Cx , I2C_EVENT_MASTER_BYTE_RECEIVED));
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

void I2C_stop(I2C_TypeDef* I2Cx)
{
	I2C_GenerateSTOP(I2Cx , ENABLE);
}


void GPIO_Config()
{
	GPIO_InitTypeDef 	GPIO_InitStruct;
	USART_InitTypeDef 	USART_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD , &GPIO_InitStruct);


	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF; //Alternatif fonksiyonlar(input,output,adc disinda baska sey oldugu)
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOA,&GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);

	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStruct);
	USART_Cmd(USART2 , ENABLE);
}


int main(void)
{
	I2C1_Config();
	GPIO_Config();

  while (1)
  {

	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	  I2C_write(I2C1, 0x6B); // write one byte to the slave // PWR_MGMT_1 register /* MPU-6050 çalistirildi */
	  I2C_write(I2C1, 0x00); // write one byte to the slave // set to zero (wakes up the MPU-6050)
	  I2C_stop(I2C1); // stop the transmission

	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	  I2C_write(I2C1, 0x1C); // write one byte to the slave //
	  I2C_write(I2C1, 0x00); // write one byte to the slave // set to zero (wakes up the MPU-6050)
	  I2C_stop(I2C1); // stop the transmission

	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	  I2C_write(I2C1, 0x3B); // write one byte to the slave
	  I2C_stop(I2C1); // stop the transmission
	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	  a = (I2C_read_nack(I2C1)); // read one byte and request another byte
	  m=(a) & (0x80);
	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	  I2C_write(I2C1, 0x3C); // write one byte to the slave
	  I2C_stop(I2C1); // stop the transmission
	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	  b = I2C_read_nack(I2C1); // read one byte and request another byte

	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	  I2C_write(I2C1, 0x3E); // write one byte to the slave
	  I2C_stop(I2C1); // stop the transmission
	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	  b1 = I2C_read_nack(I2C1); // read one byte and request another byte

	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	  I2C_write(I2C1, 0x3D); // write one byte to the slave
	  I2C_stop(I2C1); // stop the transmission
	  I2C_start(I2C1, Slave_Address<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	  a1 = (I2C_read_nack(I2C1)); // read one byte and request another byte
	  m1=(a1) & (0x80);


	  d=((a<<8) | b);
	  d1=((a1<<8) | b1);

	  d = map(d,-17000,17000,0,10);
	  d1 = map(d1,-17000,17000,0,10);

	  sprintf(str,"X=%d, ",d);
	  Usart_Puts(USART2,str);
	  sprintf(str,"Y=%d, \n",d1);
	  Usart_Puts(USART2,str);


	  	if(d>=7 && 2<d1 && d1<7 ) 	//Kirmizi
	  		{
	  		GPIO_ResetBits(GPIOD,GPIO_Pin_12);
	  		GPIO_SetBits(GPIOD,GPIO_Pin_13 | GPIO_Pin_14);
	  		}

	  	if(d<=2 && 2<d1 && d1<7 ) //Yesil
	  		{
	  	    GPIO_ResetBits(GPIOD,GPIO_Pin_13);
	  		GPIO_SetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_14);
	  		}

	  	if(d1>=7 && 2<d && d<7 ) 	//Mavi
	  		{
	  		GPIO_ResetBits(GPIOD,GPIO_Pin_14);
	  		GPIO_SetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_13);
	  		}

	  	if(d1<=2 && 2<d && d<7 )	 //Mor
	  		{
	  	    GPIO_ResetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_14 );
	  		GPIO_SetBits(GPIOD,GPIO_Pin_13);
	  		}

	    if(2<d && d<7 && 2<d1 && d1<7 ) //Beyaz
	  		{
	  		GPIO_ResetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);
	  		}


	  	  i=600000;

	  	  while(i)
	  	  i--;
  }
}


/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
