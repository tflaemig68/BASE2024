/**
 * I2C Device Scan
 * ST7735 TFT Display
 * ===========================
 *
 * Ansteuerung eines TFT Display ueber SPI.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
//#include <stm32f4xx.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <ST7735.h>
#include <RotaryPushButton.h>
#include <Balancer.h>
#include <Sensor3DG.h>
#include <step.h>

#include "i2cDevices.h"
#include "xyzScope.h"


#define i2cAddr_motL 0x61
#define i2cAddr_motR 0x60

uint8_t Acc = 6;

bool timerTrigger = false;


// Declaration  Timer1 = Main Prog
// 				ST7725_Timer delay Counter
uint32_t	Timer1 = 0UL;
uint32_t    ST7735_Timer = 0UL;
uint32_t    I2C_Timer = 0UL;
#define StepTaskTime 7


/* Private function prototypes -----------------------------------------------*/
void test_ascii_screen(void);
void test_graphics(void);

uint8_t I2C_SCAN(I2C_TypeDef *i2c, uint8_t scanAddr);
void StepperIHold(bool on);
int Regler(int Pos, float phi, float v);

int main(void)
{
/*  I2C Variables  */

	uint8_t        scanAddr = 0x7F;  //7Bit Adresse
	I2C_TypeDef   *i2c  = I2C1;
	I2C_TypeDef   *i2c2  = I2C2;

	int8_t BMA020ret = -1, LIS3DHret=-1, MPU6050ret=-1;
	uint32_t   i2cTaskTime = 50UL;
	bool LIS3DHenable = false;
	bool BMA020enable = false;
	bool MPU6050enable = false;
	bool StepLeftenable = false;
	bool StepRightenable = false;

/*  End I2C Variables  */

	char strCardID[]   = ".  .  .  .  .  .  .\0";
	char strFirmware[] = ". . .          \0";  // dummyString with NULL


	char strX[8],strY[8],strZ[8],strT[32];
	int8_t Temp, XPOS;
	int16_t XYZraw[3],XYZBMA[3],XYZMPU[3],XYZgMPU[3];
	float MPUfilt[3] = {0,0,0}, BMAfilt[3]= {0,0,0};
	float orgkFilt = 0.02, kFilt;
	int ButtPos, oldButtPos=0;
	int pos_motR=0, pos_motL=0;
	float XYZ[3], AlphaBeta[2];

	static uint8_t testmode = 1;
	uint16_t timeTMode5;

	//int testmode = 1;
   	//unsigned int r = 0;

       // Dies ist das Array, das die Adressen aller Timer-Variablen enthaelt.
       // Auch die Groesse des Arrays wird berechnet.
       uint32_t *timerList[] = { &I2C_Timer, &ST7735_Timer /*, weitere Timer */ };
       size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);


    BalaHWsetup();
    LED_red_on;

	//Inits needed for TFT Display
    // Initialisiert den Systick-Timer
	systickInit(SYSTICK_1MS);
	spiInit();
	tftInitR(INITR_REDTAB);

	//display setup
    tftSetRotation(LANDSCAPE_FLIP);
    tftSetFont((uint8_t *)&SmallFont[0]);
    tftFillScreen(tft_BLACK);

    /* initialize the rotary push button module */
    initRotaryPushButton();

    systickSetMillis(&I2C_Timer, i2cTaskTime);


    LED_red_off;

    tftPrint((char *)"I2C Scanner running \0",0,0,0);
    //tftPrint((char *)"Select I2C Connector \0",0,14,0);






    while (1)
    {
	   if (true == timerTrigger)
	   {
			systickUpdateTimerList((uint32_t *) timerList, arraySize);
	   }

	   if (isSystickExpired(I2C_Timer))
	   {
		   systickSetTicktime(&I2C_Timer, i2cTaskTime);
		   LED_green_off;


		   switch (testmode)
		   {
		   	   case 0:  //I2C Scan
		   	   {
		   		   //lcd7735_setForeground(ST7735_YELLOW);
		   		   i2cSetClkSpd(i2c,  I2C_CLOCK_100);
		   		   i2cSetClkSpd(i2c2,  I2C_CLOCK_400);
		   		   //tftPrint((char *)".  .  .  .  . \0",66,14,0);
		   		   testmode  = 1;
		   	   }
		   	   case 1:  //I2C Scan
		   	   {
		   		   LED_red_on;
		   		   if ( I2C_SCAN(i2c, scanAddr) != 0)
				   {
					   LED_red_off;
					   switch (scanAddr)
					   {
					   	   case i2cAddr_motL:
						   {
							   StepLeftenable = true;
							   tftPrint((char *)"<-Left STEP\0",0,14,0);
							   stepMotorInit(i2cAddr_motL,1);
						   }
						   break;
					   	   case i2cAddr_motR:
						   {
							   StepRightenable = true;
							   tftPrint((char *)"Right->\0",92,14,0);
							   stepMotorInit(i2cAddr_motR,0);
						   }
						   break;
					   	   case i2cAddr_RFID:
						   {
							   enableRFID = true;
							   tftPrint((char *)"RFID connected \0",0,56,0);
							   RFID_LED(i2c,true);
						   }
						   break;
						   case i2cAddr_LIDAR:
						   {
							   enableLIDAR = true;
							   //lcd7735_print((char *)"TOF/LIADR connected \0",0,28,0);
						   }
						   break;
						   case i2cAddr_LIS3DH:
						   {
							   LIS3DHenable = true;
							   tftPrint((char *)"LIS3DH connected \0",0,28,0);

							   tftPrint((char *)"Temp:\0",0,40,0);
							   tftPrint((char *)"X:\0",0,50,0);
							   tftPrint((char *)"Y:\0",0,60,0);
							   tftPrint((char *)"Z:\0",0,70,0);
							   LED_blue_on;
						   }
						   break;
						   case i2cAddr_BMA020:
						   {
							   BMA020enable = true;
							   tftPrint((char *)"BMA020 \0",0,42,0);
							   tftPrint((char *)"X:\0",0,50,0);
							   tftPrint((char *)"Y:\0",0,60,0);
							   tftPrint((char *)"Z:\0",0,70,0);
						   }
						   break;
						   case i2cAddr_MPU6050:
						   {
							   MPU6050enable = true;
							   tftPrint((char *)"MPU6050 \0",65,42,0);

						   }
						   break;
					   }
				   }

				   if ((scanAddr == 0) && (enableRFID))
				   {
					   scanAddr = 0x7F;
					   i2cTaskTime = 200UL;
					   		// SL018 only works with 100kHz
					   testmode = 2;
				   }
				   if ((scanAddr == 0) && ((LIS3DHenable)|| (BMA020enable)||(MPU6050enable)))
				   {
					   LED_blue_on;
					   scanAddr = 0x7F;
					   testmode = 4;
					   i2cTaskTime = 200;

				   }
				   if ((scanAddr == 0))
				   {
					   scanAddr = 0x7F;
					   if (i2c == I2C1)
					   {
						   i2c = I2C2;
					   }
					   else
					   {
						   i2c = I2C1;
						   tftFillScreen(tft_BLACK);
					   }
				       testmode = 0;
				   }
				   else
				   {
					   scanAddr -=1;
				   }
				   break;
				}
		   	   	case 2:  // read RFID Firmware
				{
					if (RFID_readFWVersion(i2c, (char *)strFirmware) >= 0)
					{
						tftPrint((char *)"FW: \0",0,48,0);
						tftPrint((char *)strFirmware,24,48,0);
						testmode = 3;
						tftPrint((char *)"ID:\0",0,70,0);
					}
					else
					{
						;
					}
				}
				break;
		   	   	case 3:  // read RFID ID
		   		{
		   			if (RFID_readCard(i2c, strCardID)> 0)
		   			{
		   				tftPrint((char *)strCardID,24,70,0);
		   			}
		   		}
		   		break;

// 3DG Sensor function
		   	 	case 4:  // 3DGInit Init
		   	 	{
		   	 		LED_green_on;
		   			if ((BMA020enable) && (BMA020ret < 0))
		   			{
		   				BMA020ret = i2cBMA020_init(i2c,0);
		   			}
		   			else
		   			{ BMA020ret = 0; }
		   			if ((MPU6050enable) && (MPU6050ret <0))
					{
						MPU6050ret = i2cMPU6050_init(i2c,0);
					}
		   			else
		   			{ MPU6050ret = 0; }

					if (BMA020ret > 0)										// no LIS3DH Sensor present
					{
						tftPrint("no 3DGSensors Present ",0,0,0);
						i2cTaskTime = 500;
						testmode = 1;
					}



					if ((BMA020ret == 0)  && (MPU6050ret == 0))									// LIS3DH init-procedure finished
					{
						if ((StepRightenable)&& (StepLeftenable))
						{
							setAccShape(i2cAddr_motR, 0);
							setAcceleration(i2cAddr_motR, Acc);
							//setIrun(i2cAddr_motR, Irun);
							//setIhold(i2cAddr_motR, Ihold);

							setAccShape(i2cAddr_motL, 0);
							setAcceleration(i2cAddr_motL, Acc);
							//setIrun(i2cAddr_motL, Irun);
							//setIhold(i2cAddr_motL, Ihold);
							i2cTaskTime = StepTaskTime;									// Tasktime for Stepper Balancing 70ms
							testmode = 9;
							tftFillScreen(tft_BLACK);
							tftPrint("DHBW BALANCER (c)Fl\0",0,0,0);


						}
						else
						{
							i2cTaskTime = 70;									// Tasktime for display 70ms
							testmode = 5;
							timeTMode5 = 100;							// count of cycles in Mode5
						}
					}
				}
				break;
		   	 	case 5:  // read 3DG Data
				{
					LED_green_off;
					LED_red_on;
					if (BMA020enable)
					{
						i2cBMA020XYZ(i2c,(int16_t *) XYZBMA);
						XPOS =15;
						sprintf(strX, "%+5i", XYZBMA[2]); tftPrint((char *)strX,XPOS,50,0);
						sprintf(strX, "%+5i", -XYZBMA[1]); tftPrint((char *)strX,XPOS,60,0);
				//		sprintf(strX, "%+5i", XYZBMA[0]); tftPrint((char *)strX,XPOS,70,0);
						AlphaBeta[0] =57* atan((float)-XYZBMA[1]/XYZBMA[2]);
						sprintf(strX, "%+4.1f", AlphaBeta[0]); tftPrint((char *)strX,XPOS,80,0);
					}
					if (MPU6050enable)
					{
						i2cMPU6050XYZ(i2c,(int16_t *) XYZMPU);
						XPOS =60;
						const float MPU6050Res = 16.384;
						sprintf(strX, "%+5.0f", XYZMPU[0]/MPU6050Res); tftPrint((char *)strX,XPOS,50,0);
						sprintf(strX, "%+5.0f", XYZMPU[1]/MPU6050Res); tftPrint((char *)strX,XPOS,60,0);
					//	sprintf(strX, "%+5.0f", XYZMPU[2]/MPU6050Res); tftPrint((char *)strX,XPOS,70,0);
						AlphaBeta[1] =57* atan((float)XYZMPU[1]/XYZMPU[0]);
						sprintf(strX, "%+4.1f", AlphaBeta[1]); tftPrint((char *)strX,XPOS,80,0);

						i2cMPU6050GYRO(i2c,(int16_t *) XYZgMPU);
						XPOS =115;
						const float MPU6050GyroRes = 131;
						sprintf(strX, "%+4.0f", XYZgMPU[2]/MPU6050GyroRes); tftPrint((char *)strX,XPOS,70,0);
				//		sprintf(strX, "%+4.0f", XYZgMPU[1]/MPU6050GyroRes); tftPrint((char *)strX,XPOS,60,0);
				//		sprintf(strX, "%+4.0f", XYZgMPU[2]/MPU6050GyroRes); tftPrint((char *)strX,XPOS,70,0);


					}

					if ((timeTMode5--) > 0)
					{
						testmode = 8;
						i2cTaskTime = StepTaskTime;
						LED_blue_off;
						LED_red_off;
						tftFillScreen(tft_BLACK);

					}
		   	 	}
				break;
		 	 	case 6:  // LIS3DH Init		   			   		{
				{
					LED_red_off;
					LIS3DHret = i2cLIS3DH_init(i2c, 0);
					if (LIS3DHret > 0)										// no LIS3DH Sensor present
					{
						tftPrint("LIS3DH not Present ",0,0,0);
						i2cTaskTime = 500;
						testmode = 1;
					}
					if (LIS3DHret == 0)										// LIS3DH init-procedure finished
					{
						tftPrint("(C)23Fl I2C LIS3DH ",0,0,0);
						i2cTaskTime = 70;									// Tasktime for display 70ms
						testmode = 7;
						timeTMode5 = 10;							// count of cycles in Mode5
					}
				}
				break;
		   		case 7:  // read LIS3DH Data
		   		{
		   			LED_blue_on;

		   			Temp = i2cLIS3DH_Temp(i2c);
		   			sprintf(strT, "%+3i", Temp);
		   			tftPrint((char *)strT,40,40,0);

		   			i2cLIS3DH_XYZ(i2c,(int16_t *) XYZraw);

  					XYZ[0] = (float) XYZraw[0]/0x3FFF;  //skalierung 1mg/digit at +-2g
		   			XYZ[1] = (float) XYZraw[1]/0x3FFF;
		   			XYZ[2] = (float) XYZraw[2]/0x3FFF;
		   			sprintf(strX, "%+6.3f", XYZ[0]);
		   			tftPrint((char *)strX,20,50,0);
		   			sprintf(strY, "%+6.3f", XYZ[1]);
		   			tftPrint((char *)strY,20,60,0);
		   			sprintf(strZ, "%+6.3f", XYZ[2]);
		   			tftPrint((char *)strZ,20,70,0);
					if ((timeTMode5--) > 0)
					{
						testmode = 8;
						tftFillScreen(tft_BLACK);
						tftPrint("T:    LIS3DH (C)23Fl",0,0,0);
						i2cTaskTime = 100;
						LED_blue_off;

					}
				    break;
				}
		   		case 8:  // Scope display the LIS3DH Data
				{
					if (BMA020enable)
					{
						i2cBMA020XYZ(i2c,(int16_t *) XYZBMA);
						AlphaBeta[0] = atan((float)-XYZBMA[1]/XYZBMA[2]);
					}
					if (MPU6050enable)
					{
						i2cMPU6050XYZ(i2c,(int16_t *) XYZMPU);
						AlphaBeta[1] = atan((float)XYZMPU[1]/XYZMPU[0]);
					}

					//i2cLIS3DH_XYZ(i2c, XYZraw);

					if (fabs(AlphaBeta[1]) < 0.1)
					{
						setRotaryColor(LED_GREEN);
					}
					else
					{
						setRotaryColor(LED_YELLOW);
					}
					AlphaBeta[0] *=10;
					AlphaBeta[1] *=10;
					AlBeScreen(AlphaBeta);



					//testmode = 2;

				}
				break;
		   		case 9:  // StepperPosition folgt dem Neigungswinkel
				{
					if (BMA020enable)
					{
						i2cBMA020XYZ(i2c,(int16_t *) XYZBMA);
						getFiltertAccData(XYZBMA, BMAfilt, kFilt);
						AlphaBeta[0] = atan((float)-BMAfilt[1]/BMAfilt[2]);
					}
					if (MPU6050enable)
					{
						i2cMPU6050XYZ(i2c,(int16_t *) XYZMPU);
						getFiltertAccData(XYZMPU, MPUfilt, kFilt);
						AlphaBeta[1] = atan(MPUfilt[1]/MPUfilt[0]);
					}

					//i2cLIS3DH_XYZ(i2c, XYZraw);

					if (fabs(AlphaBeta[1]) < 0.1)
					{
						setRotaryColor(LED_GREEN);
					}
					else
					{
						setRotaryColor(LED_YELLOW);
					}
					if (fabs(AlphaBeta[1]) > 0.7)  //ca pi/4
					{
						setRotaryColor(LED_MAGENTA);
						StepperIHold(false);
						softStop(i2cAddr_motR);
						softStop(i2cAddr_motL);
						resetPosition(i2cAddr_motR);
						pos_motR = 0;
						resetPosition(i2cAddr_motL);
						pos_motL = 0;
					}
					else
					{
						StepperIHold(true);
						pos_motL =(int)(AlphaBeta[0]*573);
						pos_motR =(int)(AlphaBeta[1]*573);
						if (StepRightenable)
						{
							setPosition(i2cAddr_motR, pos_motR);
							StepRightenable = false;
						}
						else
						{
							setPosition(i2cAddr_motL, pos_motL);
							StepRightenable = true;
						}
					}
					ButtPos = getRotaryPosition();
					if (getRotaryPushButton())
					{
						tftPrintInt(ButtPos,120,20,0);
						tftPrintFloat(AlphaBeta[0],0,34,0);
						tftPrintFloat(AlphaBeta[1],100,34,0);
					}

					if (ButtPos != oldButtPos)
					{
						kFilt = orgkFilt+ ((float)ButtPos)/-500;
						if (kFilt < 0.001) {kFilt = 0.001;}
						if (kFilt > 1) {kFilt =1;}


						sprintf(strT, "kFilt %5.3f ", kFilt);
						tftPrint((char *)strT,10,20,0);
						oldButtPos = ButtPos;
					}

					//testmode = 2;

				}
				break;
		   		default:
				{
					testmode = 0;
				}
		   }  //end switch (testmode)
	   } // end if systickexp
    } //end while
    return 0;
}

void StepperIHold(bool on)
{
	static bool status_off = false;
	const uint8_t Ihold = 10;
	const uint8_t Ioff = 0;
	if (on == status_off)
	{
		if (on)
		{
			setIhold(i2cAddr_motL,Ihold);
			setIhold(i2cAddr_motR,Ihold);
		}
		else
		{
			setIhold(i2cAddr_motL,Ioff);
			setIhold(i2cAddr_motR,Ioff);
		}
	}

}



/* scanAdr. 7Bit Adresse value
 * return	0 if no device found on scanAdr
 *			if yes  return the scanAdr.
 *			and display on the ST7735 Display
 *
 *
 */



uint8_t I2C_SCAN(I2C_TypeDef *i2c, uint8_t scanAddr)
{
	uint8_t 	*outString2 = (uint8_t *) "Addr at: \0";
	uint8_t     port, *result;
#define yPosBase 28
	uint8_t foundAddr = 0;
	static int xPos[2] = {0,100};
	static int yPos[2] = {yPosBase, yPosBase};

	if (i2c == I2C1)
    {
	   port = 0;
    }
    else
    {
	   port = 1;
    }
    if (scanAddr == 0)
    {
    yPos[0] = yPosBase;
    yPos[1] = yPosBase;
    }

	foundAddr = i2cFindSlaveAddr(i2c, scanAddr);
	if (yPos[port] == 0)
	{
		tftPrint((char *)outString2,xPos[port],yPos[port],0);
		yPos[port] = 66;
	}
	result = convDecByteToHex(scanAddr);
	if (foundAddr != 0)
	{
		//outString = outString2;
		tftPrint((char *)result,xPos[port],yPos[port],0);
		yPos[port] = (int) 14 + yPos[port];
		if (yPos[port] > 100)
		{
			yPos[port] = yPosBase;
		}
	}
	else
	{
	//	tftPrint((char *)result,xPos,14,0);
	}
	return foundAddr;

}

#define ParamCount 5
struct Parameter
{
	char Title[5];
	float Value;
	float Min;
	float Max;
	float manInc;
} Param[ParamCount];


char ParamTitle[ParamCount][5] = {"Phi0","TP3dg","KP","KD","Rot"};
//Param[0].Value = 0;



struct RegPameter
{
	float phi_0;			// Winkel der neutralen Nulllage
	float tp_3dg;			// Faktor des Tiefpass der Sensorwerte
	float tp_tar;			// Tiefpass ??
	float KP;				// Proportional Faktor [steps/phi]
	float KD;				// Differential Faktor
	float Rotation;			// Eigenrotatio Steps pro Zeitslot
} RegPa;

int Regler(int Pos, float phi, float v)
{
	static float phi_old =0;

	float delta_phi = phi - RegPa.phi_0;
	int _iTargetPos = (int)( RegPa.KP* delta_phi + (RegPa.KD* tan(phi - phi_old)));
	phi_old = phi;
	return (Pos+_iTargetPos);
}

/*
void balanceMotor(void)
{
	const int offset_phi = 0; 		// Absolutwert des Winkels für die Schwerpunktlage in degr*10
	int tp_fakttar = 50, tp_fakt3dg = 23;
	int kp = 180;//345; // 140;
	int kd = 999;	//1200 ;			// P Anteil kp/1000 , D-Anteil kd/1000
	int y_off = 100;
	//const float deg2rad = 0.0001745;				// Faktor PI/180°/100
	static int y_old, PotPos;
	static BYTE Ihold, OnMot;
	static long _sto_ltargmean;
	BYTE ret;
	long _ltargetpos;
	int _itargetpos, filt_target, x,y,z, PotPos_raw, phi, rw, Pos_OK, _txyz[4], gxyz[4];
	int rot_l, rot_r;
	static int pos_motL= 0, pos_motR= 0;
	int disp = 0;
	rot_l = -turnSteps;
	rot_r = turnSteps;


	//PotPos =  Conv_mV(ADCfilt[Chan_Uin])/(maxUpot_mV/250) - 125;
	y_off += PotPos;
	//kd += 3*PotPos;
	ret = read_axes(D3Sens_addr, _txyz);
	low_pass(_txyz, gxyz, tp_fakt3dg);
//----------------------------------------------------------------------------
// Regler auf Basis der Achs-Beschleunigungen
	y = gxyz[2];
	z = gxyz[3];
	sprintf(senden, "y%+04i,z%+04i", y,z);
	y += y_off;
	_ltargetpos = (((long)y* (long)kp + (long)(kd*(y - y_old))))/((long)z);
	y_old = y;

	_sto_ltargmean += (long)(_ltargetpos) - (filt_target = _sto_ltargmean/tp_fakttar);
	_itargetpos= -(int)_ltargetpos;  // Richtungsumkehr
//----------------------------------------------------------------------------
	sprintf(senden2, "tr%+05i,mt%+05i", _itargetpos, filt_target);
//	phi = (int)phi_yz(gxyz)-offset_phi+PotPos;
//	rw = (phi/10)*(phi_old/10);		// Werteüberlauf vermeiden
//	if (rw >= 0) // true kein Vorzeichenwechsel,d.h. kein Seitenwechsel


//		if ((phi < 150) && (phi > -150))
//		{
//			_itargetpos = (int) (kp/2* (float) tan(((float)phi)*deg2rad));		// Ruckelvermeidung durch geringere Verstärkung
//		}
//		else
//		{
//			_itargetpos = (int) (kp* (float) tan(((float)phi)*deg2rad));
//		}
//	_itargetpos += (int) (kd*(float) (phi - phi_old)*deg2rad);
	//_itargetpos += (int) (kd*(float) tan(((float)(phi - phi_old))*deg2rad));
//	phi_old = phi;



	Pos_OK = 0;
	if (z > 500)
	{
		Pos_OK = 1;
	} // <Pos nur wenn noch innerhalb von 40° gekippt ist
	else
	{
		softStop(motR_addr);
		softStop(motL_addr);
		sprintf(senden2, " 1___ %03i ---v ", y_off);
		resetPosition(motR_addr);
		pos_motR = 0;
		resetPosition(motL_addr);
		pos_motL = 0;
	}




//	if ((phi < 4500) && (phi > -4500)) {Pos_OK = 1;}		 // <Pos nur wenn noch innerhalb von 45° gekippt ist
//	if ((Pos_OK == 1) && (OnMot != 1))
//	{
//		setIhold(motL_addr,Ihold);
//		setIhold(motR_addr,Ihold);
//		OnMot = 1;
//	}
//	if (Pos_OK != 1)
//	{
//		Ihold = getIhold(motL_addr);
//		setIhold(motL_addr,0);
//		setIhold(motR_addr,0);
//		OnMot = 0;
//	}

	if (Pos_OK == 1)
	{
//			resetPosition(motL_addr);
//			setPosition(motL_addr, _itargetpos+rot_l);
//			resetPosition(motR_addr);
//			setPosition(motR_addr, _itargetpos+rot_r);


		if (_itargetpos <= rot_l )
		{

			// pos_motR = getActualPosition(motR_addr);		//
			setPosition(motR_addr, pos_motR+_itargetpos+rot_r);
			pos_motR += _itargetpos+rot_r;
		}
		else
		{
			//pos_motR = getActualPosition(motR_addr);		//
			setPosition(motR_addr, pos_motR+_itargetpos);
			pos_motR += _itargetpos;
		}
		if (rot_r <= _itargetpos)
		{
			//pos_motL = getActualPosition(motL_addr); //
			setPosition(motL_addr, pos_motL+_itargetpos+rot_l);
			pos_motL += _itargetpos+rot_l;
		}
		else
		{
			//pos_motL = getActualPosition(motL_addr); //
			setPosition(motL_addr, pos_motL+_itargetpos);
			pos_motL += _itargetpos;
		}
	}

}

*/



