/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "usbd_cdc_if.h"//giao tiep usb
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <Kalman.h>
#include "lcd16x2.h"
#include "goc_pin.h"
#include "mpu6050_kalman.h"
#include "mppt.h"
extern char ReceivedData[100];//mang du lieu nhan USB
extern char SendData[100];//mang du lieu gui
extern uint8_t Rxcount;
extern uint32_t dataSize;
extern uint8_t check;//kiem tra nhan data USB
extern uint32_t time;

char receiveuart[100]="",senduart[100]="";//giao tiep uart
uint8_t rxdata='\0',uartbuf[100],rx=0,tx=1,uartcount=0,uartsize=0;

#define K1_Port GPIOB
#define K2_Port GPIOB

#define chieu_Port GPIOA
#define nguon_Port GPIOA
char data[100];//mang du lieu xuat lcd

RTC_TimeTypeDef sTime;
RTC_DateTypeDef DateToUpdate;
uint8_t sec;//bien lay thoi gian thuc
uint8_t min;
uint8_t hour;
uint8_t wday;
uint8_t date;
uint8_t month;
uint8_t year;
//uint16_t c1,c2,c;
uint8_t shour=12,smin=12,ssec=0,sdate=1,smonth=1,syear=21;//bien set thoi gian

float VD=21.033;//vi do
float KD=105.85;//kinh do
extern int n;//so thu tu ngay trong nam
extern int PALT,PATT,TALT,TATT;//PALT, PATT goc cuc ly thuyet,thuc te
 
//chuong trinh doc cam bien mpu6050		
extern Kalman_t kalmanX;//cau truc kalman
extern Kalman_t kalmanY;
extern double accX, accY, accZ;
extern double gyroX, gyroY, gyroZ;
extern int16_t tempRaw;

extern double gyroXangle, gyroYangle; // Angle calculate using the gyro only
extern double compAngleX, compAngleY; // Calculated angle using a complementary filter
extern double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

// dieu khien xy lanh
int goc_max=165,goc_min=15,tmax=17,tmin=7,PA=90,PAS=90;
//goc_max,goc_min gioi han hanh trinh
//tmax, tmin gioi han thoi gian lam viec
//PA goc cuc can dieu khien PAS goc cuc dat che do tay
float deltaT=1.0,deltaA=3.0,t=-1.0;
//t thoi gian khoi tao ban dau
// deltaT(h) khoang thoi gian dat lai goc PA
// deltaA sai lech goc cho phep
uint8_t mode=0;//che do hoat dong xy lanh
uint8_t m=0;// m=0 che do dieu khien tu dong ,m=1 che do dieu khien tay
void dk_xl()
{	
  PATT=kalAngleY+90.0;
  if(m==1) {
		 PA=PAS;
			if(PATT<PA) //quay thuan
		 {
				HAL_GPIO_WritePin(nguon_GPIO_Port,nguon_Pin,1);
				HAL_GPIO_WritePin(chieu_GPIO_Port,chieu_Pin,0);
				mode=1;
		 }
		else if(PATT>(PA+deltaA)) //quay nguoc
		 {
				HAL_GPIO_WritePin(nguon_GPIO_Port,nguon_Pin,1);
				HAL_GPIO_WritePin(chieu_GPIO_Port,chieu_Pin,1);
				mode=2;
		 }
		else if((PATT>=PA)&&(PATT<=(PA+deltaA)))
		 {
				HAL_GPIO_WritePin(nguon_GPIO_Port,nguon_Pin,0);	
				HAL_GPIO_WritePin(chieu_GPIO_Port,chieu_Pin,0);	
				mode=0;
		 }
		 else {}
	}
	if(m==0){
	 if (((hour+min/60.0)>=tmin)&&((hour+min/60.0)<=tmax))
	 { 
			if(fabs(hour+min/60.0-t)>=deltaT)
			{ // dat lai goc quay sau thoi gian deltaT
			goc_pin(year,month,date,hour,min,sec,VD,KD);
			PA=PALT; // dat goc ly thuyet
			t=hour+min/60.0;
			}
			if(PA<goc_min) PA=goc_min;//gioi han goc dat PA
			if(PA>goc_max) PA=goc_max;
		if(PATT<PA) //quay thuan
		 {
				HAL_GPIO_WritePin(nguon_GPIO_Port,nguon_Pin,1);
				HAL_GPIO_WritePin(chieu_GPIO_Port,chieu_Pin,0);
				mode=1;
		 }
		else if(PATT>(PA+deltaA)) //quay nguoc
		 {
				HAL_GPIO_WritePin(nguon_GPIO_Port,nguon_Pin,1);
				HAL_GPIO_WritePin(chieu_GPIO_Port,chieu_Pin,1);
				mode=2;
		 }
		else if((PATT>=PA)&&(PATT<=(PA+deltaA)))//dung
		 {
				HAL_GPIO_WritePin(nguon_GPIO_Port,nguon_Pin,0);	
				HAL_GPIO_WritePin(chieu_GPIO_Port,chieu_Pin,0);	
				mode=0;
		 }
		 else  {}
	  }
  else if (((hour+min/60.0)>tmax)&&((hour+min/60.0)<=tmax+0.25))
		{ 
		if(PATT>goc_min){
				HAL_GPIO_WritePin(nguon_GPIO_Port,nguon_Pin,1);
				HAL_GPIO_WritePin(chieu_GPIO_Port,chieu_Pin,1);
				mode=3;
				}
		else{
				HAL_GPIO_WritePin(nguon_GPIO_Port,nguon_Pin,0);	
				HAL_GPIO_WritePin(chieu_GPIO_Port,chieu_Pin,0); 
				mode=4;
			 }
		}
	else
    {
				HAL_GPIO_WritePin(nguon_GPIO_Port,nguon_Pin,0);	
				HAL_GPIO_WritePin(chieu_GPIO_Port,chieu_Pin,0);
			  mode=4;
	  }
	}
}

//do dien ap dong dien
uint16_t adc_data[6];//bien luu gia tri adc
float V_PV,I_PV,V_AQ,I_AQ,I_T;//bien do dong ap
void do_dong_ap()
{ 
	float adc0=0,adc1=0,adc2=0,adc3=0,adc4=0;
	for (int i=0;i<100;i++) adc0=adc0+adc_data[0];
	adc0=adc0/100.0;
	for (int i=0;i<100;i++) adc1=adc1+adc_data[1];
	adc1=adc1/100.0;
	for (int i=0;i<100;i++) adc2=adc2+adc_data[2];
	adc2=adc2/100.0;
	for (int i=0;i<100;i++) adc3=adc3+adc_data[3];
	adc3=adc3/100.0;
	for (int i=0;i<100;i++) adc4=adc4+adc_data[4];
	adc4=adc4/100.0;
	V_PV=(0.0000003*adc0*adc0+ 0.0003*adc0 + 0.0129)*(100+5.6)/5.6+0.5;
//	I_PV=(adc1*(3.33/4096.0)-2.4)/0.1;
//	V_AQ=(0.0000003*adc2*adc2+ 0.0003*adc2 + 0.0129)*(100+5.6)/5.6-3;
//	I_AQ=(adc3*(3.33/4096.0)-2.4)/0.1;
//	I_T=(adc4*(3.33/4096.0)-2.4)/0.1;

//	V_PV=(-0.000000009*adc0*adc0+ 0.0009*adc0 - 0.0335)*(100+5.6)/5.6;
	I_PV=(-(0.000000002*adc1*adc1+ 0.0008*adc1 + 0.0046)+2.445)/0.1+2;
	V_AQ=(0.000000002*adc2*adc2+ 0.0008*adc2 + 0.0046)*(100+5.6)/5.6-3;
	I_AQ=(-(0.000000002*adc3*adc3+ 0.0008*adc3 + 0.0046)+2.445)/0.1+1;
	I_T=(-(0.000000002*adc4*adc4+ 0.0008*adc4 + 0.0046)+2.445)/0.1+0.1;
	
//	V_PV=adc_data[0]*2.7/(4095.0);
//	I_PV=(adc_data[1]*(2.7/4095.0)-2.5)/0.1;
//	V_AQ=adc_data[2]*3.4/(4095.0);
//	I_AQ=(adc_data[3]*(3.4/4095.0)-2.5)/0.1;
//	I_T=(adc_data[4]*(3.4/4095.0)-2.5)/0.1;
}

//hien thi LCD
int32_t timedisp1=-2000,timedisp2=-2000;//tg hien thi
void do_tinh_hien_thi()
{
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
		uint32_t dateToStore;
		memcpy(&dateToStore,&DateToUpdate,4);
		BKP->DR2 = dateToStore >> 16;
		BKP->DR3 = dateToStore & 0xffff;
    sec=sTime.Seconds;
    min=sTime.Minutes;
    hour=sTime.Hours;
    wday=DateToUpdate.WeekDay;
    date=DateToUpdate.Date;
    month=DateToUpdate.Month;
    year=DateToUpdate.Year;
	if((double)(HAL_GetTick()-timedisp1)/1000.0>=2){//hien thi sau 1s
	  goc_pin(year,month,date,hour,min,sec,VD,KD);
    PATT=kalAngleY+90;
		LCD_Init();
    LCD_Gotoxy(0,0);
		sprintf(data,"%0.2d:%0.2d|%0.3d|%0.3d      ",hour,min,PALT,PATT);
		LCD_Puts(data);
		LCD_Gotoxy(0,1);
		sprintf(data,"%0.2d/%0.2d/%0.2d|%0.3d       ",date,month,year,n);
		LCD_Puts(data);
		if(m==0){LCD_Gotoxy(14,0);LCD_Puts("M0");}//auto
		if(m==1){LCD_Gotoxy(14,0);LCD_Puts("M1");}//manual
		if(mode==0){LCD_Gotoxy(13,1);LCD_Puts("XL0");}//dung
		if(mode==1){LCD_Gotoxy(13,1);LCD_Puts("XL1");}//thuan
		if(mode==2){LCD_Gotoxy(13,1);LCD_Puts("XL2");}//nguoc
		if(mode==3){LCD_Gotoxy(0,1);LCD_Puts("TIME OUT     XL2");}//chi co mode auto
		if(mode==4){LCD_Gotoxy(0,1);LCD_Puts("TIME OUT     XL0");}
		timedisp1=HAL_GetTick();
		}
	if(((double)(HAL_GetTick()-timedisp1)/1000.0>=1)&&(double)(HAL_GetTick()-timedisp2)/1000.0>=2){
		do_dong_ap();
		LCD_Clear();
		sprintf(data,"%2.2f %2.2f    " ,V_PV,V_AQ);
		LCD_Gotoxy(0,0);
		LCD_Puts(data);
		sprintf(data,"%2.1f %2.1f %2.1f       " ,I_PV,I_AQ,I_T);
		LCD_Gotoxy(0,1);
		LCD_Puts(data);
		timedisp2=HAL_GetTick();
	}
}

//dieu khien sac
extern float duty;
//float V_PVmax=47.8,V_AQmax=60,V_AQon=50.4,V_AQoff=48,I_AQmax=6,V_AQfloat=56.4,I_Tmax=6;
float V_PVmax=22.84,V_AQmax=30,V_AQon=25.2,V_AQoff=24,I_AQmax=5,V_AQfloat=28.2,I_Tmax=5;
int32_t timewait=-5000;
void dk_sac()
{
	if((double)(HAL_GetTick()-timewait)/1000.0>=5)
	{
		timewait=-5000;
		do_dong_ap();
		if((V_PV<V_AQ)&&(V_PV<V_PVmax))
		{
			HAL_GPIO_WritePin(K1_GPIO_Port,K1_Pin,1);//noi PV
			if((V_AQ<V_AQmax)&&(I_AQ<I_AQmax))
			{
				MPPT_INC(V_PV,I_PV);
				//MPPT_PO(V_PV,I_PV);
				//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,MPPT_INC(V_PV,I_PV)*1439);
			}
			if(V_AQ==V_AQfloat)
			{
				//duy tri duty
			}
		}
		if((V_PV>V_PVmax)||(I_AQ>I_AQmax))
		{
			HAL_GPIO_WritePin(K1_GPIO_Port,K1_Pin,0);//ngat PV
			timewait=HAL_GetTick();
		}
		if(V_AQ>V_AQon)
		{
			HAL_GPIO_WritePin(K2_GPIO_Port,K2_Pin,1);//noi tai
		}
		if((V_AQ<V_AQoff)||(I_T>I_Tmax))
		{
			HAL_GPIO_WritePin(K2_GPIO_Port,K2_Pin,0);//ngat tai
			timewait=HAL_GetTick();
		}
 }
}

//cat_chuoi
char  **cat_chuoi(char*s1,char*s2)
{
		int i=0;
		char **s3;//=(char **)malloc(sizeof(char));
		s3[i] = strtok(s1,s2); 
		while(s3[i] != NULL)
		{
			i++;
			//s3=(char **)realloc(s3,sizeof(char)*(i+1));
			s3[i] = strtok(NULL, s2);	
		}
		return s3;
		//free(s3);
}

//giam sat SCADA
uint8_t p=0,g=0;
char*idgate="g0";
char*idnode="n0";
void SCADA(char*idgate,char*idnode)
{
    if(check == 1) //neu co du lieu truyen den
		{
			 CDC_Transmit_FS((uint8_t *)ReceivedData, strlen(ReceivedData));//gui du lieu vua nhan
			 char **str;//=(char **)calloc(1000,sizeof(char));
			 str=cat_chuoi(ReceivedData,"|");//cat chuoi du lieu luu mang str=[idgate,idnode,action,data]
			if((!strcmp(str[0],idgate))&&(!strcmp(str[1],idnode)))
			{
				 if(!strcmp(str[2],"0"))//cai dat thoi gian
					 {
						 shour=atoi(str[3]);
						 smin=atoi(str[4]);
						 ssec=atoi(str[5]);
						 sdate=atoi(str[6]);
						 smonth=atoi(str[7]);
						 syear=atoi(str[8]);

						sTime.Seconds=ssec;
						sTime.Minutes=smin;
						sTime.Hours=shour;
						//DateToUpdate.WeekDay=2;
						DateToUpdate.Date=sdate;
						DateToUpdate.Month=smonth;
						DateToUpdate.Year=syear;
						HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
						HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
					 }
				 else if(!strcmp(str[2],"1"))//cai dat vi tri
					 {					 
						 VD=(float) atof(str[3]);
						 KD=(float) atof(str[4]);
						 sprintf(data,"% 3.3f-%3.3f   " ,VD,KD);
						 LCD_Gotoxy(0,1);
						 LCD_Puts(data);
						 HAL_Delay(100);
					 }
				 else if(!strcmp(str[2],"2"))//thiet lap che do
					 {
						 t=-1.0;
						 m=atoi(str[3]);
						 PAS=(float) atof(str[4]);
					 }
				 else if(!strcmp(str[2],"4"))//phan hoi gate
					 {
						 p=atoi(str[3]);
					 }
				 else if(!strcmp(str[2],"3"))//gui data
					 {
						 g=atoi(str[3]);
						 sprintf(SendData,"%s|%s|%s|%.2f|%.2f|%.2f|%0.2f|%0.2f|%0.0f|\n\r" ,idgate,idnode,"3",V_PV,I_PV,V_AQ,I_AQ,I_T,kalAngleY+90.0);
						 CDC_Transmit_FS((uint8_t *)SendData, strlen(SendData));
						 HAL_UART_Transmit_IT(&huart1,(uint8_t*)SendData,strlen(SendData));
					 }
				else
         {
//					 	 g=atoi(str[3]);
//						 sprintf(SendData,"%s|%s|%s|%.2f|%.2f|%.2f|%0.2f|%0.2f|%d|\n\r" ,str[0],str[1],"3",V_PV,I_PV,V_AQ,I_AQ,I_T,PATT);
//						 CDC_Transmit_FS((uint8_t *)SendData, strlen(SendData));
//						 HAL_UART_Transmit_IT(&huart1,(uint8_t*)SendData,strlen(SendData));
         }
			 }
				 check = 0; //free(str);
		}
    if(rx == 1) //neu co du lieu truyen den
		{
		   //sprintf(receiveuart,"%s\n\r",receiveuart);
			 CDC_Transmit_FS((uint8_t *)receiveuart, strlen(receiveuart));
			 char **str;//=(char **)calloc(1000,sizeof(char));
			 str=cat_chuoi(receiveuart,"|");//cat chuoi du lieu luu mang str=[idgate,idnode,action,data]
			if((!strcmp(str[0],idgate))&&(!strcmp(str[1],idnode)))
			{
				 if(!strcmp(str[2],"0"))//cai dat thoi gian
					 {
						 shour=atoi(str[3]);
						 smin=atoi(str[4]);
						 ssec=atoi(str[5]);
						 sdate=atoi(str[6]);
						 smonth=atoi(str[7]);
						 syear=atoi(str[8]);

						sTime.Seconds=ssec;
						sTime.Minutes=smin;
						sTime.Hours=shour;
						//DateToUpdate.WeekDay=2;
						DateToUpdate.Date=sdate;
						DateToUpdate.Month=smonth;
						DateToUpdate.Year=syear;
						HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
						HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
					 }
				 else if(!strcmp(str[2],"1"))//cai dat vi tri
					 {					 
						 VD=(float) atof(str[3]);
						 KD=(float) atof(str[4]);
						 sprintf(data," %3.3f-%3.3f   " ,VD,KD);
						 LCD_Gotoxy(0,1);
						 LCD_Puts(data);
						 HAL_Delay(100);
					 }
				 else if(!strcmp(str[2],"2"))//thiet lap che do
					 { 
//						 int mm;
						 t=-1.0;
						 m=atoi(str[3]);
//						 if (mm==1) m=1;
//						 else if (mm==0) m=0;
//						 else {}
						 PAS=(float) atof(str[4]);
					 }
				 else if(!strcmp(str[2],"4"))//phan hoi gate
					 {
						 p=atoi(str[3]);
					 }
				 else if(!strcmp(str[2],"3"))//gui data
					 {
						 g=atoi(str[3]);
						 sprintf(senduart,"%s|%s|%s|%.2f|%.2f|%.2f|%0.2f|%0.2f|%0.0f|\n\r" ,idgate,idnode,"3",V_PV,I_PV,V_AQ,I_AQ,I_T,kalAngleY+90.0);
						 CDC_Transmit_FS((uint8_t *)senduart, strlen(senduart));
						 HAL_UART_Transmit_IT(&huart1,(uint8_t*)senduart,strlen(senduart));
					 }
				else 
				{
//						 g=atoi(str[3]);
//						 sprintf(senduart,"%s|%s|%s|%.2f|%.2f|%.2f|%0.2f|%0.2f|%d|\n\r" ,str[0],str[1],"3",V_PV,I_PV,V_AQ,I_AQ,I_T,PATT);
//						 CDC_Transmit_FS((uint8_t *)senduart, strlen(senduart));
//						 HAL_UART_Transmit_IT(&huart1,(uint8_t*)senduart,strlen(senduart));
				}
			 }
			  rx=0;//free(str);
		}
}
//float rate,comprate,gyrrate,aa,bb;
//uint32_t tt;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc_data,6);
	HAL_UART_Receive_IT(&huart1,&rxdata,1);
  LCD_Init();
	do_dong_ap();
	MPPT_INIT(V_PV,I_PV);
	init_kalman_mpu6050();
  for(int i=0;i<=300;i++){kalman_mpu6050();}
	//HAL_GPIO_WritePin(K1_GPIO_Port,K1_Pin,1);
	//HAL_GPIO_WritePin(K2_GPIO_Port,K2_Pin,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(fabs(kalAngleY+90.0-PATT)>=20)
		{
			MX_I2C2_Init();
			init_kalman_mpu6050();
			//for(int i=0;i<=300;i++){kalman_mpu6050();}
		}
		kalman_mpu6050();
//  rate = gyroY / 131.0; // Convert to deg/s
//	gyrrate =(gyroYangle-aa)/(HAL_GetTick()-tt)*1000;
//	comprate =(compAngleY -bb)/(HAL_GetTick()-tt)*1000;
//	aa=gyroYangle;bb=compAngleY;tt=HAL_GetTick();
		
		do_tinh_hien_thi();
		dk_xl();
		dk_sac();
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty*1439);
		//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,3600);
		//MPPT(V_PV,I_PV);
		SCADA(idgate,idnode);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
	if( HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) !=0x32F2)
{
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 12;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_FRIDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 21;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
}
 
else
{
 uint32_t dateMem;
 dateMem = BKP->DR3;
 dateMem |= BKP->DR2 << 16;
 memcpy(&DateToUpdate,&dateMem,sizeof(uint32_t));
 if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
 {
 Error_Handler();
 }
 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
}
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1439;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, chieu_Pin|nguon_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, K1_Pin|K2_Pin|D7_Pin|D6_Pin 
                          |D5_Pin|D4_Pin|EN_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : chieu_Pin nguon_Pin */
  GPIO_InitStruct.Pin = chieu_Pin|nguon_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : K1_Pin K2_Pin D7_Pin D6_Pin 
                           D5_Pin D4_Pin EN_Pin RS_Pin */
  GPIO_InitStruct.Pin = K1_Pin|K2_Pin|D7_Pin|D6_Pin 
                          |D5_Pin|D4_Pin|EN_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14 
                           PB15 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance)
	{
		//if(uartcount==0){for(int i=0;i<=uartsize;i++)buf[i]=0;}
		if(rxdata==13||rxdata==10){
				sprintf(receiveuart,"%s\n\r",uartbuf);
			  //memcpy(receiveuart,uartbuf,sizeof(uint8_t)*(uartcount-1));
				uartsize=uartcount;
				rx=1;uartcount=0;
			}
		else uartbuf[uartcount++]=rxdata;
	}
		HAL_UART_Receive_IT(&huart1,&rxdata,1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	tx=1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
