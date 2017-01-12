#include <math.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_spi.h>
#include <stm32f0xx_dma.h>
#include <stm32f0xx_misc.h>

//I2S GPIO definitions
#define I2S_WS		GPIO_Pin_4
#define I2S_CK		GPIO_Pin_5
#define I2S_MCK		GPIO_Pin_6
#define I2S_SD		GPIO_Pin_7

#define I2S_WSPS	GPIO_PinSource4
#define I2S_CKPS	GPIO_PinSource5
#define I2S_MCKPS	GPIO_PinSource6
#define I2S_SDPS	GPIO_PinSource7

#define I2S_AF		GPIO_AF_0
#define I2S_GPIO	GPIOA
#define I2S_SPI		SPI1

//DMA Buffer size, this can be adjusted if samples seem to be dropped
#define DMA_BUFSIZ	32

//Sampling frequency
//#define FS			48000
//Sampling frequency with error correction - 48000*(100-2.3438)/100 = 46874.98Hz
#define FS			46875

//Waveform output frequency (subject to 2.34% error due to PLL)
#define FREQOUT		8000

//DMA Buffer
int16_t dmabuf[DMA_BUFSIZ*2] = {0};

//Sinewave wavetable - constant population would reduce SRAM requirements!
int16_t sinewt[256];

//Peripheral typedefs
GPIO_InitTypeDef G;
I2S_InitTypeDef I;
DMA_InitTypeDef D;
NVIC_InitTypeDef N;

//Array population function
void Populate(uint32_t pos){
	//tw = Tuning word, waves are generated using DDS: http://interface.khm.de/index.php/lab/interfaces-advanced/arduino-dds-sinewave-generator/
	//FS multiplied by two as buffer actually contains both left and right!
	const uint32_t tw = (4294967296UL/(2*FS))*FREQOUT;
	static uint32_t phac = 0, sinph, cosph;

	int16_t sample;
	uint32_t n;

	for(n = pos; n<pos+DMA_BUFSIZ; n++){
		//Sine wave phase
		sinph = phac>>(32-8);

		//Cosine wave phase
		//Note the addition of 256/4 (64) as a cosine wave is 1/4 of a cycle ahead of a sine wave. The anding with 255
		//ensures the wavetable index wraps
		cosph = (sinph + 256/4)&255;

		//Every even buffer sample is for the left hand channel
		if(n&1){
			//Right
			sample = sinewt[sinph];
		}
		else{
			//Left
			sample = sinewt[cosph];
		}

		//Write sample to dma buffer
		dmabuf[n] = sample;

		//Increment phase accumulator
		phac += tw;
	}
}

//DMA interrupt handler
void DMA1_Channel2_3_IRQHandler(void){
	//Once the first half of the buffer has been sent, populate the first half (during
	//this time, the second half will be being sent!)
	if(DMA_GetITStatus(DMA1_IT_HT3)){
		DMA_ClearITPendingBit(DMA1_IT_HT3);
		Populate(0);
	}
	else if(DMA_GetITStatus(DMA1_IT_TC3)){
		DMA_ClearITPendingBit(DMA1_IT_TC3);
		//After the second half has been sent, re-populate while the first half is being
		//sent.
		Populate(DMA_BUFSIZ);
	}
}

int main(void)
{
	//Enable required clocks
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	//Initialize pins
	G.GPIO_Pin = I2S_WS | I2S_CK | I2S_MCK | I2S_SD;
	G.GPIO_Mode = GPIO_Mode_AF;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_PuPd = GPIO_PuPd_NOPULL;
	G.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2S_GPIO, &G);

	//Intialize I2S peripheral
	I.I2S_AudioFreq = I2S_AudioFreq_48k;
	I.I2S_CPOL = I2S_CPOL_Low;
	I.I2S_DataFormat = I2S_DataFormat_16b;
	I.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
	I.I2S_Mode = I2S_Mode_MasterTx;
	I.I2S_Standard = I2S_Standard_Phillips;
	I2S_Init(I2S_SPI, &I);
	SPI_I2S_DMACmd(I2S_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

	//Initialize DMA peripheral
	D.DMA_BufferSize = DMA_BUFSIZ*2;
	D.DMA_DIR = DMA_DIR_PeripheralDST;
	D.DMA_M2M = DMA_M2M_Disable;
	D.DMA_MemoryBaseAddr = (uint32_t) dmabuf;
	D.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	D.DMA_MemoryInc = DMA_MemoryInc_Enable;
	D.DMA_Mode = DMA_Mode_Circular;
	D.DMA_PeripheralBaseAddr = (uint32_t)&(I2S_SPI->DR);
	D.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	D.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	D.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel3, &D);

	//Configure DMA interrupts
	DMA_ClearITPendingBit(DMA1_IT_HT3);
	DMA_ClearITPendingBit(DMA1_IT_TC3);
	DMA_ITConfig(DMA1_Channel3, DMA_IT_HT, ENABLE);
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

	N.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
	N.NVIC_IRQChannelPriority = 0;
	N.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&N);

	//Generate sine wavetable
	uint16_t n;
	for(n = 0; n<256.0; n++){
		//16bit wavetable, 2^(16-1)-1 ~= +32767 to -32767
		sinewt[n] = 32767*sin((double)n*2*M_PI/256.0);
	}

	//Enable DMA and I2S
	DMA_Cmd(DMA1_Channel3, ENABLE);
	I2S_Cmd(I2S_SPI, ENABLE);

    while(1)
    {
    }
}
