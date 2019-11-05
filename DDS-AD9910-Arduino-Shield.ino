/* Includes ------------------------------------------------------------------*/

#include "AD9910.h"

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ClickButton.h>

#define MODE_PIN     A0
#define DOWN_PIN     A1
#define UP_PIN       A2
#define LOW_FREQ_LIMIT  100000 
#define HIGH_FREQ_LIMIT  490000000 


int M, K, H, A, MenuPos;

ClickButton modeButton(MODE_PIN, LOW, CLICKBTN_PULLUP);
ClickButton upButton(UP_PIN, LOW, CLICKBTN_PULLUP);
ClickButton downButton(DOWN_PIN, LOW, CLICKBTN_PULLUP);

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

uint32_t kley;
uint16_t Amplitude_ramp_rate = 0xFFFF;
uint16_t Amplitude_scale_factor = 0x3FFF;
uint8_t Amplitude_step_size = 0;
int16_t Amplitude_dB = -30; //16127
void setup()
{
  Serial.begin(115200);
  DDS_Init();
  M=300;
  K=000;
  H=0;
  A=0;
  MenuPos=0;
  modeButton.debounceTime   = 75;   // Debounce timer in ms
  modeButton.multiclickTime = 1;  // Time limit for multi clicks
  modeButton.longClickTime  = 1000; // time until "held-down clicks" register

  upButton.debounceTime   = 75;   // Debounce timer in ms*/
  upButton.multiclickTime = 1;  // Time limit for multi clicks*/
  upButton.longClickTime  = 1000; // time until "held-down clicks" register*/

  downButton.debounceTime   = 70;   // Debounce timer in ms
  downButton.multiclickTime = 1;  // Time limit for multi clicks
  downButton.longClickTime  = 1000; // time until "held-down clicks" register

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
 
  UpdateDisplay();
}


/*************************************************************************
 * Freq Out, freq_output in Hz
 ************************************************************************/
void Freq_Out(uint32_t freq_output, int16_t amplitude_dB_output ) {
  DDS_Fout(&freq_output, amplitude_dB_output, Single_Tone_Profile_5);
 
  HAL_GPIO_WritePin(DDS_PROFILE_0_GPIO_PORT, DDS_PROFILE_0_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DDS_PROFILE_1_GPIO_PORT, DDS_PROFILE_1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PROFILE_2_PIN, GPIO_PIN_SET);

}
 

void loop ()
{

    /* USER CODE BEGIN 2 */
/*	
	kley = 133300617;
  DDS_Fout(&kley, Single_Tone_Profile_0);
	
	kley = 100000000;
  DDS_Fout(&kley, Single_Tone_Profile_1);
	
	kley = 150000000;
  DDS_Fout(&kley, Single_Tone_Profile_2);
	
	kley = 200000000;
  DDS_Fout(&kley, Single_Tone_Profile_3);
	
	kley = 250000000;
  DDS_Fout(&kley, Single_Tone_Profile_4);
	
	kley = 300000000;
  DDS_Fout(&kley, Single_Tone_Profile_5);
	
	kley = 350000000;
  DDS_Fout(&kley, Single_Tone_Profile_6);
	
	kley = 133300617;
  DDS_Fout(&kley, Single_Tone_Profile_7);
	
	HAL_GPIO_WritePin(DDS_PROFILE_0_GPIO_PORT, DDS_PROFILE_0_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DDS_PROFILE_1_GPIO_PORT, DDS_PROFILE_1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PROFILE_2_PIN, GPIO_PIN_SET);
	*/
	
//	HAL_GPIO_WritePin(DDS_PROFILE_0_GPIO_PORT, DDS_PROFILE_0_PIN, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(DDS_PROFILE_1_GPIO_PORT, DDS_PROFILE_1_PIN, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PROFILE_2_PIN, GPIO_PIN_SET);
	
	
//	HAL_GPIO_WritePin(DDS_PROFILE_0_GPIO_PORT, DDS_PROFILE_0_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DDS_PROFILE_1_GPIO_PORT, DDS_PROFILE_1_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DDS_PROFILE_2_GPIO_PORT, DDS_PROFILE_2_PIN, GPIO_PIN_RESET);
	
	
//	DDS_RAM(&strSQUARE_WAVE[0]);
//	DDS_FM(100000000, 1000, 1000);
	
//	DDS_RAM(&strFM[0], 4);

  int functionUpButton=0;
  int functionDownButton=0;

  char strBuffer1[10];
 
  strBuffer1[0]=0x30;
  strBuffer1[1]=0x31;
  strBuffer1[2]=0x32;
  strBuffer1[3]=0x33;;
  strBuffer1[4]=0x34;
  while (1)
  {
    modeButton.Update();
    upButton.Update();
    downButton.Update();

    if (upButton.clicks != 0) functionUpButton = upButton.clicks;

    if ((functionUpButton == 1 && upButton.depressed == false) ||
        (functionUpButton == -1 && upButton.depressed == true))
    {
      //if (MenuPos==0) {M=Inc(M); if (M>HIGH_FREQ_LIMIT/1000000) M=420;}
      if (MenuPos==0) {if (Check (M+1, K, H)) M=Inc(M);}
      if (MenuPos==1) {if (Check (M, K+1, H)) K=Inc(K);}
      if (MenuPos==2) {if (Check (M, K, H+1)) H=Inc(H);}
      if (MenuPos==3) 
      {
        A=A+1;
        if (A>85) A=85;
      }
      UpdateDisplay();
    } 
    if (upButton.depressed == false) functionUpButton=0;


    if (downButton.clicks != 0) functionDownButton = downButton.clicks;

    if ((functionDownButton == 1 && downButton.depressed == false) ||
        (functionDownButton == -1 && downButton.depressed == true))
    {
      //if (MenuPos==0) {M=Dec(M); if (M>HIGH_FREQ_LIMIT/1000000) M=420;}
      if (MenuPos==0) {if (Check(M-1, K, H)) M=Dec(M);}
      if (MenuPos==1) {if (Check(M, K-1, H)) K=Dec(K);}
      if (MenuPos==2) {if (Check(M, K, H-1)) H=Dec(H);}
      if (MenuPos==3) 
      {
        A=A-1;
        if (A<0) A=0;
      }
      UpdateDisplay();
    }
    if (downButton.depressed == false) functionDownButton=0;

    if (modeButton.clicks > 0)
    {
      MenuPos++;
      if (MenuPos>3) MenuPos=0;
      UpdateDisplay();
    }
    
   Amplitude_dB = A;
    Freq_Out(M*1000000L + K*1000L + H, Amplitude_dB*-1);

   // ASF_SET(Amplitude_ramp_rate, Amplitude_scale_factor, Amplitude_step_size);
    /*HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(7, (uint8_t*)strBuffer1, 5, 1000);
   HAL_GPIO_WritePin(DDS_SPI_CS_GPIO_PORT, DDS_SPI_CS_PIN, GPIO_PIN_SET);*/
   //DDS_Init();
    //delay(1);
  }
  /* USER CODE END 3 */

}

void UpdateDisplay()
{
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  display.println(F("DDS AD9910"));
  
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 16);     // Start at top-left corner

  display.println(F("Frequency, [Hz]:"));

  display.setTextSize(2);      // Normal 1:1 pixel scale
  if (MenuPos==0) display.setTextColor(BLACK, WHITE); 
    else display.setTextColor(WHITE);
  display.setCursor(1, 26);   

  //display.println(F("420000000"));
  display.print(PreZero(M));
  if (MenuPos==1) display.setTextColor(BLACK, WHITE); 
    else display.setTextColor(WHITE);
  display.print(PreZero(K));
  if (MenuPos==2) display.setTextColor(BLACK, WHITE); 
    else display.setTextColor(WHITE);
  display.println(PreZero(H));

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 42);     // Start at top-left corner

  display.println(F("Amplitude:"));

  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(1, 50);     // Start at top-left corner

  if (MenuPos==3) display.setTextColor(BLACK, WHITE); 
    else display.setTextColor(WHITE);
  display.print("-");  
  display.print(PreZero(A));
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(48, 56);
  display.println("dBM");

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); 
  display.setCursor(70, 56);     // Start at top-left corner
  
  display.println(F("GRA&AFCH"));
  display.display();
}

String PreZero(int Digit)
{
  if ((Digit<100) && (Digit>=10)) return "0"+String(Digit);
  if (Digit<10) return "00"+String(Digit);
  return String(Digit);
}

int Inc(int val)
{
  uint32_t FreqVal=M*1000000 + K*1000 + H;
  //Serial.println(FreqVal);
  //if (FreqVal>=420000000) return val;
  val++;
  if (val>999) val=999;
  return val;
}

int Dec(int val)
{
 uint32_t FreqVal=M*1000000 + K*1000 + H;
 //Serial.println(FreqVal);
 //if (FreqVal<=100000) return val;
 val--;
 if (val<0) val=0;
 return val;
}


/*****************************************************************
 * 
 * **************************************************************/


bool Check (int _M, int _K, int _H)
{
  long F_Val;
  F_Val = _M*1000000L + _K*1000L + _H;
//  Serial.print("_M=");
//  Serial.println(M);
//  Serial.print("_K=");
//  Serial.println(_K);
//  Serial.print("_H=");
//  Serial.println(_H);
//  Serial.print("F_val=");
//  Serial.println(F_Val);
  if((F_Val >= LOW_FREQ_LIMIT) && (F_Val <= HIGH_FREQ_LIMIT)) return 1;
    else return 0;  
}
  
