/******************************************************************************
 * AD9910 - DDS Power Supply 3.3V � 1.8V (China board)
 * Internal PLL 1GHz, REF CLK Oscillator 40Mhz (I do not recommend using) N=25, 
 * For More pure spectrum Recomendeted replace VCTCXO 638CGP-10-3 10Mhz N=100
 * For signal without harmonics and the spur is required at the output of 
 * +IOUT & -IOUT to use a transformer or ADT1-1WT ADT2 + LPF 400MHZ 7rd order.
 * Overclocking the Internal PLL to 1.1 GHz, REF CLK = 10Mhz(VCTCXO 638CGP-10-3), N = 110
 * Fout 500 Mhz without LPF
 * 05.08.2017
 * Author Grisha Anofriev e-mail: grisha.anofriev@gmail.com
******************************************************************************/

//#include "stm32f4xx_hal.h"
#include "ad9910.h"
//#include "square_wave.h"
//#include <stm32f4xx_hal_spi.h>
//#include <stm32f4xx_hal_gpio.h>
#include <math.h>
#include <float.h>
#include <Arduino.h>
int hspi1=0;

   
/* Private variables ---------------------------------------------------------*/

uint32_t DAC_Current;
uint32_t DAC_Current_L;
uint32_t DAC_Current_H;
uint64_t ADDR_FTW;


uint8_t ADDR_FTW_L;
uint8_t ADDR_FTW_H;
uint16_t ADDR_RAM;
uint8_t kley7;
uint8_t kley6;
uint8_t kley5;
uint8_t kley4;
uint8_t kley3;
uint8_t kley2;
uint8_t kley1;

uint16_t NumCellsInArray; 
//uint16_t NumRAM_Words;
uint8_t  CMD;

uint8_t strBuffer[9]={129, 165, 15, 255};
uint8_t strBufferRAM[1000]={0,0,0};  // was 
uint8_t strBufferREAD[1000]={0,0,0}; // was 
uint8_t strBufferReceiver[1000]={0}; // was 


//uint32_t kley;
uint32_t FTW;
uint32_t *jlob;

double T_step;
uint64_t Step_Rate;
uint16_t STEP;
uint16_t n;
double DEG;
double RAD;
double strK_SIN[1024] ={0};
uint32_t strFREQ_FM[1024] ={0};
uint32_t strFTW_FM[1024] ={0};


void HAL_SPI_Transmit(int *blank, uint8_t *strBuffer, int nums, int pause)
{
  for (int i=0;i<nums; i++)
  {
    SPI.transfer(*(strBuffer+i));
  }
}

void HAL_Delay (int del)
{
  delayMicroseconds(del);
}
/******************************************************************************
 * HAL to Arduino
******************************************************************************/

void HAL_GPIO_WritePin (int port, int pin, int mode)
{

  digitalWrite(pin, mode);
}



/******************************************************************************
 * Init GPIO for DDS
******************************************************************************/
void DDS_GPIO_Init(void)
{
   pinMode(DDS_SPI_SCLK_PIN, OUTPUT);
   pinMode(DDS_SPI_SDIO_PIN, OUTPUT);
   pinMode(DDS_SPI_SDO_PIN, INPUT);
   pinMode(DDS_SPI_CS_PIN, OUTPUT);
   pinMode(DDS_IO_UPDATE_PIN, OUTPUT);
   pinMode(DDS_IO_RESET_PIN, OUTPUT);
   pinMode(DDS_MASTER_RESET_PIN, OUTPUT);
   pinMode(DDS_PROFILE_0_PIN, OUTPUT);
   pinMode(DDS_PROFILE_1_PIN, OUTPUT);
   pinMode(DDS_PROFILE_2_PIN, OUTPUT);
   pinMode(DDS_OSK_PIN, OUTPUT);
   pinMode(DDS_TxENABLE_PIN, OUTPUT);
   pinMode(DDS_F0_PIN, OUTPUT);
   pinMode(DDS_F1_PIN, OUTPUT);
   pinMode(DDS_DRHOLD_PIN, OUTPUT);
   pinMode(DDS_PWR_DWN_PIN, OUTPUT);
   pinMode(DDS_DRCTL_PIN, OUTPUT);
   
   pinMode(DDS_DROVER, INPUT);
   pinMode(DDS_SYNC_CLK, INPUT);
   pinMode(DDS_RAM_SWP_OVR, INPUT);
   pinMode(DDS_PLL_LOCK, INPUT);
   pinMode(DDS_PDCLK_PIN, INPUT);
     

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DDS_IO_UPDATE_GPIO_PORT, DDS_IO_UPDATE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DDS_MASTER_RESET_GPIO_PORT, DDS_MASTER_RESET_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DDS_OSK_GPIO_PORT, DDS_OSK_PIN, GPIO_PIN_RESET);                     // OSK = 0
	HAL_GPIO_WritePin(DDS_PROFILE_0_GPIO_PORT, DDS_PROFILE_0_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DDS_PROFILE_1_GPIO_PORT, DDS_PROFILE_1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PROFILE_2_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_DRHOLD_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_DRCTL_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PWR_DWN_PIN, GPIO_PIN_RESET);

	 
}

/******************************************************************************
 * Init SPI, 8bit, Master
 * MODE 3, MSB, 
******************************************************************************/
void DDS_SPI_Init(void)
{
  /*hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }*/

  SPI.begin(); //
  SPI.setDataMode (SPI_MODE0); 
  SPI.setClockDivider(SPI_CLOCK_DIV8); //16MHZ/8=2MHZ
  SPI.setBitOrder(MSBFIRST);
  

}


/*****************************************************************************
  Initialization DDS
  * Config SPI, Reset DDS and ReConfig SPI
  * Enable/Disable internal PLL, mux, div, charge pump current, Set VCO
  * Set Current Output
*****************************************************************************/
void DDS_Init(void)
 {
	 DDS_GPIO_Init();
	 
	 	// It is very important for DDS AD9910 to set the initial port states
	 HAL_GPIO_WritePin(DDS_MASTER_RESET_GPIO_PORT, DDS_MASTER_RESET_PIN, GPIO_PIN_SET);   // RESET = 1
	 HAL_GPIO_WritePin(DDS_MASTER_RESET_GPIO_PORT, DDS_MASTER_RESET_PIN, GPIO_PIN_RESET); // RESET = 0
	 HAL_GPIO_WritePin(DDS_IO_UPDATE_GPIO_PORT, DDS_IO_UPDATE_PIN, GPIO_PIN_RESET);       // IO_UPDATE = 0	 
   HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);               // CS = 1
	 HAL_GPIO_WritePin(DDS_OSK_GPIO_PORT, DDS_OSK_PIN, GPIO_PIN_SET);                     // OSK = 1
	 HAL_GPIO_WritePin(DDS_PROFILE_0_GPIO_PORT, DDS_PROFILE_0_PIN, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(DDS_PROFILE_1_GPIO_PORT, DDS_PROFILE_1_PIN, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PROFILE_2_PIN, GPIO_PIN_RESET);
	 
	 DDS_SPI_Init();
	 
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET); 	 
	 strBuffer[0] = CFR1_addr;
	 strBuffer[1] = 0;// RAM_enable;//RAM_Playback_Amplitude;// | RAM_enable;//0x00; 
	 strBuffer[2] = //Inverse_sinc_filter_enable;//0; //Continuous_Profile_0_1; //0;//0x80;//0x00;
	 strBuffer[3] = 0; //OSK_enable | Select_auto_OSK;//0x00;
	 strBuffer[4] = SDIO_input_only ;
   HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 5, 1000);
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	 DDS_UPDATE();
	 
	 
	 
	 
	 
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET); 	 
	 strBuffer[0] = CFR2_addr;
	 strBuffer[1] = Enable_amplitude_scale_from_single_tone_profiles;//1;//0x00;
	 strBuffer[2] = 0;//SYNC_CLK_enable;// | Read_effective_FTW;
	 strBuffer[3] = 0;//0x08;//PDCLK_enable;
	 strBuffer[4] = Sync_timing_validation_disable;// | Parallel_data_port_enable;
   HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 5, 1000);
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	 DDS_UPDATE();
	 
	  
	 
	 
	 
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET); 	 
	 strBuffer[0] = CFR3_addr;
	 strBuffer[1] = VCO5;// | DRV0_REFCLK_OUT_High_output_current;
	 strBuffer[2] = Icp287uA;
	 strBuffer[3] = REFCLK_input_divider_ResetB | PLL_enable; // REFCLK_input_divider_bypass; //
	 strBuffer[4] = N100; // SYSCLK= REF_CLK * N
   HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 5, 1000);
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	 DDS_UPDATE();
	
	
	
		HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET); 	 
	 strBuffer[0] = FSC_addr;
	 strBuffer[1] = 0;
	 strBuffer[2] = 0;
	 strBuffer[3] = 0;
	 strBuffer[4] = 0x7F; // Max carrent 255 = 31mA
   HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 5, 1000);
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
   DDS_UPDATE();

	
	 
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET); 	 
	 strBuffer[0] = 0x0E;
	 strBuffer[1] = 0x3F;
	 strBuffer[2] = 0xFF;
	 strBuffer[3] = 0x00;
	 strBuffer[4] = 0x00;
	 
	 strBuffer[5] = 0x19;
	 strBuffer[6] = 0x99;
	 strBuffer[7] = 0x99;
	 strBuffer[8] = 0x9A; // 100mhz
   HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 9, 1000);
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	 DDS_UPDATE(); 
	 
	/*
	
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET); 	 
	 strBuffer[0] = FTW_addr;
	 strBuffer[1] = 0x0C;//0x19;
	 strBuffer[2] = 0xCC;//0x99;
	 strBuffer[3] = 0xCC;//0x99;
	 strBuffer[4] = 0xCD;//50mhz//0x9A; // 100mhz
   HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 5, 1000);
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	 
   DDS_UPDATE();
	 
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET); 	 
	 strBuffer[0] = ASF_addr;
	 strBuffer[1] = 0xFF;
	 strBuffer[2] = 0xFF;
	 strBuffer[3] = 0x01;
	 strBuffer[4] = 0x0F;
   HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 5, 1000);
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	 
   DDS_UPDATE();
	 
	*/ 
	 

 }                           

 
/*****************************************************************************************
   Update - data updates from memory
*****************************************************************************************/ 
void DDS_UPDATE(void)
{
	// Required - data updates from memory
	 HAL_Delay(10);
	 HAL_GPIO_WritePin(DDS_IO_UPDATE_GPIO_PORT, DDS_IO_UPDATE_PIN, GPIO_PIN_RESET); // IO_UPDATE = 0
	 HAL_Delay(10);
	 HAL_GPIO_WritePin(DDS_IO_UPDATE_GPIO_PORT, DDS_IO_UPDATE_PIN, GPIO_PIN_SET); // IO_UPDATE = 1
	 HAL_Delay(10);
	 HAL_GPIO_WritePin(DDS_IO_UPDATE_GPIO_PORT, DDS_IO_UPDATE_PIN, GPIO_PIN_RESET); // IO_UPDATE = 0
	 HAL_Delay(10);
}
 

/*****************************************************************************************
   ASF— Amplitude Scale Factor
*****************************************************************************************/
void ASF_SET (uint16_t *Amplitude_ramp_rate1, int16_t Amplitude_scale_factor1, uint8_t Amplitude_step_size1)
{

   HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET);    
   strBuffer[0] = ASF_addr; // 
   strBuffer[1] = 0xFF;
   strBuffer[2] = 0xFF;
   strBuffer[3] = 0xFF;
   strBuffer[4] = 0xFF;
    
   HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 5, 1000);
   HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
   DDS_UPDATE(); 
}

/*****************************************************************************************
   F_OUT - Set Frequency in HZ 
   Num_Prof - Single Tone Mode 0..7
   ASF_Val - Amplitude 14bit 0...16127
*****************************************************************************************/
void DDS_Fout (uint32_t *F_OUT, int16_t ASF_Val, uint8_t Num_Prof)
{
	 //FTW = *F_OUT;
	 FTW = ((uint32_t)(4294967296.0 *(((double)*F_OUT / F_SYSCLK)))); 
	
   jlob = & FTW;	

  // (uint16_t)10^(ASF_Val+84.288)/20;
   
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET); 	 
	 strBuffer[0] = Num_Prof; // Single_Tone_Profile_0;
  
	 strBuffer[1] =  (uint16_t)powf(10,(ASF_Val+84.288)/20.0) >> 8; ///ASF_Val>>8;//(20*log (ASF_Val/16128))>>8;
	 strBuffer[2] =  (uint16_t)powf(10,(ASF_Val+84.288)/20.0);          //ASF_Val;//20*log (ASF_Val/16128);
	 strBuffer[3] = 0x00;
	 strBuffer[4] = 0x00;
	 
	 strBuffer[5] = *(((uint8_t*)jlob)+ 3);
	 strBuffer[6] = *(((uint8_t*)jlob)+ 2);
	 strBuffer[7] = *(((uint8_t*)jlob)+ 1);
	 strBuffer[8] = *(((uint8_t*)jlob));
		
   HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 9, 1000);
	 HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	 DDS_UPDATE(); 
	 
}	


/*****************************************************************************************
   FM sine formation
   Input data:
   * F_carrier - frequency carrier Hz
   * F_mod - frequency modulations Hz
   * F_dev - frequency deviations Hz
*****************************************************************************************/  
void DDS_FM (uint32_t F_carrier, uint32_t F_mod, uint32_t F_dev)
{
	T_step = 1/(F_mod * 1000); // necessary time step
	Step_Rate = (T_step * F_SYSCLK)/4; // the value of M for the register Step_Rate, for the desired sampling rate from RAM
		
	//*************************************************************************
  STEP = 1000; // DEG = 360/STEP = 360/1000 = 0.36 DEG for 1 STEP
  DEG = 0; // start set deg
  for (n = 0; n < STEP; ++n)
    {
     RAD = DEG * 0.01745; // conversion from degrees to radians RAD=DEG*Pi/180
     strK_SIN[n] = sin(RAD); // Get Sinus
		 strFREQ_FM[n] = F_carrier + (F_dev * strK_SIN[n]);
     strFTW_FM[n] = (uint32_t)(4294967296.0 * (strFREQ_FM[n] / F_SYSCLK)); 			
     DEG = DEG + 360.0/STEP; // 
    }
//*************************************************************************
	DDS_RAM(&strFTW_FM[0], STEP); // RAM recording and playback
}


/*****************************************************************************************
   RAM - RAM recording and playback from memory only for AD9910

 * BUFF_RAM_DDS - pointer to the first element in the array, example &strFM[0]
 * NumRAM_Words - number of words to read from the array, range 1...1024, examle 4

 An example of a function call DDS_RAM(&strFM[0], 4);
 uint32_t strFM[] = {
 0x6666666,	// 25Mhz
 0x0ccccccd, // 50Mhz
 0x1999999a, // 100Mhz++++++-9

 0x26666666  // 150Mhz };
*****************************************************************************************/
void DDS_RAM (uint32_t *BUFF_RAM_DDS, uint16_t NumRAM_Words)
{
	
	/*** Config RAM Playback ***/		
	strBuffer[0] = RAM_Profile_0; // Address reg profile 0
	strBuffer[1] = 0x00; 
	strBuffer[2] = 0x00;  // Step Rate [15:8]
	strBuffer[3] = 0x0F;  // Step Rate [7:0]
	 
	strBuffer[4] = 0x00;  // End RAM address [9:2] bits in register 15:8 bit 0x1F 1024
	strBuffer[5] = 0xC0;  // End RAM address [1:0] bits in register 7:6 bit  0xC0 
	 
	strBuffer[6] = 0x00;  // Start RAM address [9:2] bits in register 15:8 bit
	strBuffer[7] = 0x00;  // Start RAM address [1:0] bits in register 7:6 bit
		
	strBuffer[8] =  Continuous_recirculate; // b100 - Continuous recerculate   No_dwell_high |
		
		
  HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 9, 1000);
	HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	DDS_UPDATE(); 
	
	/*** Select Profile 0 ***/
  HAL_GPIO_WritePin(DDS_PROFILE_0_GPIO_PORT, DDS_PROFILE_0_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DDS_PROFILE_1_GPIO_PORT, DDS_PROFILE_1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PROFILE_2_PIN, GPIO_PIN_RESET);	
	
	
	/*** Write RAM  ***/		
	strBufferRAM[0] = RAM_Start_Word; // Start Address RAM;	
	
	//NumRAM_Words = 4;
	NumCellsInArray = NumRAM_Words * 4;

	for(ADDR_RAM = 0; ADDR_RAM < NumCellsInArray; ADDR_RAM=ADDR_RAM + 4) // 
		{		
		strBufferRAM[ADDR_RAM+1] =	*(((uint8_t*)BUFF_RAM_DDS)+ ADDR_RAM + 3);
		strBufferRAM[ADDR_RAM+2] =	*(((uint8_t*)BUFF_RAM_DDS)+ ADDR_RAM + 2);	
		strBufferRAM[ADDR_RAM+3] =	*(((uint8_t*)BUFF_RAM_DDS)+ ADDR_RAM + 1);
		strBufferRAM[ADDR_RAM+4] =	*(((uint8_t*)BUFF_RAM_DDS)+ ADDR_RAM);		
		}
	
	HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET);
 
	HAL_SPI_Transmit(&hspi1, (uint8_t*)strBufferRAM, NumCellsInArray+1, 1000);
	HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	DDS_UPDATE(); 
		

		
  PlaybackRAM(RAM_enable, RAM_Playback_Frequency);

	 


while(1);
	

  


while(1)
{
  Read_DDS(0x16, 16);
}

}



/*****************************************************************************************
   Playback from RAM DDS
    - 
    - 
*****************************************************************************************/
void PlaybackRAM (uint8_t RAM_enable_MODE, uint8_t RAM_Playback_MODE)
{
/*** First - Read data from CFR1 ***/	 
	Read_DDS(CFR1_addr, 4); // 

/*** Second - Modifying and writing data to the register ***/
  HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET); 	 
	strBuffer[0] = CFR1_addr;
	strBuffer[1] = (strBufferReceiver[0]& 0x1F) | RAM_Playback_MODE | RAM_enable_MODE; 
	strBuffer[2] = strBufferReceiver[1];  //0; //Continuous_Profile_0_1; //0;//0x80;//0x00;
	strBuffer[3] = strBufferReceiver[2];  //0;//OSK_enable | Select_auto_OSK;//0x00;
	strBuffer[4] = strBufferReceiver[3];  //SDIO_input_only ;
  //HAL_SPI_Transmit(&hspi1, (uint8_t*)strBuffer, 5, 1000);
  SPI.transfer(strBuffer[0]);
  SPI.transfer(strBuffer[1]);
  SPI.transfer(strBuffer[2]);
  SPI.transfer(strBuffer[3]);
  SPI.transfer(strBuffer[4]);
	HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);
	 
	DDS_UPDATE();	
		
}


//uint8_t RAM_enable_MODE,
//	                uint8_t RAM_Playback_MODE, 
//									uint8_t Manual_OSK_external_control_MODE,
//	                uint8_t Continuous_Profile_MODE, 
//									uint8_t OSK_enable_MODE)


/*****************************************************************************************
   Read from DDS
    - ADR_DDS - address register for DDS
    - NUM_BYTE - the number of bytes to read from DDS memory
*****************************************************************************************/
void Read_DDS (uint8_t ADR_DDS, uint16_t NUM_BYTE)
{
	CMD = SPI_Read | ADR_DDS;
	HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET);
	
	//HAL_SPI_Transmit(&hspi1, &CMD, 1, 1000);
  SPI.transfer(CMD);
	//HAL_SPI_Receive(&hspi1, &strBufferReceiver[0], NUM_BYTE, 1000);
	
	HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);	
} 



/***********************************************************************************
  SPI 32 bit
  * 8bit Instruction Word
  * 8bit Data 0...255 Current 0...31,25mA
************************************************************************************/
void DDS_Current(uint8_t DAC_Current)
{  

}


/**************************************************************
Tabl Value DAC to real dBm
**************************************************************/
uint16_t strdBm[] = {
	25, // -7dBm
  60, // -6dBm
  130, //-5dBm
  195, //-4dBm
  265, //-3dBm
  325, //-2dBm
  435, //-1dBm
  510, // 0dBm
  620, //+1dBm
  740, //+2dBm
  875, //+3dBm
  1015}; //+4dBm
