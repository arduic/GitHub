/*
 * File created by Patrick Cross. Created 3/4/2013 Finished N/A
 *
 * This file while called test file will likely house all of the code for the quad copter just in unusable chunks. Not sure yet.
 * So far it can do UART Transmission and PWM output
 * Next on the list is UART in and I2C com
 */



/*
 * The other ultrasonic may be used. Distance = ((Duration of high level)*(Sonic :340m/s))/2
 * I might want to look into making a seperate file from main and including it. Then have the systick stuff be the only thing inside of main
 * Things that need to be looked into / implemented
 * putting the device into sleep mode.
 * ADC readings
 * Bluetooth resets
 * Something fun for the hell of it idk
 * The actual main program
 * Auto landing feature
 * Hover mode
 * Lift off
 * Stream video using a GoPro
 */


/**********************************************************************************************
 *								Include files
 *********************************************************************************************/
#include "math.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"

#include "driverlib/sysctl.h"	//Setup for the stellaris system, things like enabeling the clock and such
#include "driverlib/gpio.h"  	//Enables general input output
#include "driverlib/timer.h"	//Enables very specific time communications
#include "driverlib/pin_map.h"	//Maps peripheral pins to GPIO pins
#include "driverlib/rom_map.h"
#include "driverlib/adc.h"		//Setup for the ADC pins
#include "driverlib/i2c.h"		//Setup for the I2C coomunication
#include "driverlib/uart.c"		//Setup the UART for the serial com because serial doesn't have stop bits
#include "driverlib/fpu.h"		//Allows for floating point inside interrupts later on. Needed likely
#include "driverlib/systick.h"	//This is used for basic timing so it can interrupt when the timer is up. Also it is needed for the gyro
#include "driverlib/watchdog.h"	//This includes the watchdog timer

#include "float.h"

/**********************************************************************************************
 *								Global Variables
 *********************************************************************************************/
unsigned long ulPeriod = 256;

char asleep = 0;		//This var will store if the system is asleep or not
//I should look into passing these by refrence. Would be much smoother
short recievedCommands[12];			//For the same reason here I will only be reading chars in here BUT the char get function can return -1 which would overflow the char
short commandAddress=-2;			//Can not use byte they are unsigned by default
//char recievedPitchVal=0;					These are the chars in the packet order expected to be stored in the array
//char recievedRollVal=0;
//char recievedYawVal=0;
//char recievedThrottle
//char recievedSpecialCommand=0;

//Offsets for the gyroscope. CHIP SPECIFIC!
char g_offx = 66;
char g_offy = 41;
char g_offz = 12;

//Some variables for the inclination equations, could possibly be local not sure
char firstSample = 1;
long lastTime = 0;
long interval = 0;

int timeBetweenCalculations=5;		//Time in ms
float PI=3.1415926;
float wGyro = 10.0;

char caseyIsPoop = 0;


//So this is the I2C data but not sure if I want this stuff here or somewhere else....
unsigned long quadAcc[6];
unsigned long quadGyro[8];
float RwAcc[3];
float Gyro_ds[3];
float RwGyro[3];
float Awz[2];
float RwEst[3];
float pitch_error, roll_error, integral_pitch, integral_roll, derivative_pitch, derivative_roll = 0;

//This is because UART needs to be reset sometimes. Sadly I can't find what the data type sizes are in CCS but this is sure to work inefficiently.
long UARTCount = 0;

/**********************************************************************************************
 *								Prototyping
 *********************************************************************************************/
//This function will set the PWM for the desired timer and thus desired pin on off for duty cycle from 0-0xFF note OFF not ON.
//We do not need to disable and enable the timer before calling timer match set it will just update the value note after 0xBF ESC is off
inline void PWMSet(unsigned long timer, unsigned long dutyLow);

//Return the value on ADC still in construction	Probably need to fix the interrupt setup
unsigned long ADCRead();

//This will be used to handle button presses calling for a bluetooth reset
void onBoardInteruptHandle(void);

//Send a single char over UART if multiple chars, it is the calle's responsibility to handle including burst send settings
inline void UARTSend(char trans);

//UART Int Handler. Will handle the data recieved and put in an array. WILL NOT UNDERSTAND JUST STORE
void Uart2IntHandler(void);

//This will setup the com for a device, declare the regiester to write to and then write to that regiester
//Note I2C waits for a response bit before continuing. If there is no device on the other end the masterbusy loop will NEVER exit
void I2CTransmit(unsigned char device, unsigned char regiester, unsigned char value);

//Read from device using address and put the read values into buff. num=num of bytes to read
void I2CRead(unsigned char device, unsigned char regiester, unsigned char num, unsigned long *buff);

//Convert ACC readings into g force values
void rawAccToG(unsigned long *raw, float *RwAcc);

//Convert Gyro readings to Deg/sec
void rawGyroToDegsec(unsigned long *raw, float *Gyro_ds);

//Converts all the gyro acc data into usable data for roll pitch yaw... eventually
void getInclination(float *RwAcc, float *RwEst, float *RwGyro, float *Gyro_ds, float *Awz);

//The main loop that the program goes through on time intervals
void SysTickIntHandler(void);

//The watchdog timer for when the device hangs, currently used to help I2C along
void WatchdogIntHandler(void);

//This will reset the bluetooth connection, during this time the Stellaris will be stuck using old data so flight is mildly dangerous.
//In future events I need to make the Stellaris go into hover mode when these things are happening.
void ResetBluetooth(void);

void ResetBluetooth(void);

float floatingSquare(float x);

float floatingAdd(float x, float y);

//Used for detecting voltage issues
void voltageCutoffHanlder(void);

/**********************************************************************************************
 *								PWM Functions
 *********************************************************************************************/
//This function will set the PWM for the desired timer and thus desired pin on off for duty cycle from 0-0xFF note OFF not ON.
//We do not need to disable and enable the timer before calling timer match set it will just update the value note after 0xBF ESC is off
inline void PWMSet(unsigned long timer, unsigned long dutyLow){
	//TimerPrescaleMatchSet(timer,TIMER_A,(dutyLow << 8));
	TimerMatchSet(timer,TIMER_A,(dutyLow << 8));		//Shift the data 8 bit's because the clock is shifted 8 bits
	//SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1);
	//unsigned long bob = TimerMatchGet(timer,TIMER_A);
	//unsigned long jim = TimerLoadGet(timer,TIMER_A);

}

/**********************************************************************************************
 *								On Board Interrupt
 *********************************************************************************************/
//This will be used to handle button presses calling for a bluetooth reset
void onBoardInteruptHandle(void){
	//double I2C1_BIT_DELAY=(SysCtlClockGet() / (100 * 3));	//100us
	GPIOPinIntClear(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);
	ResetBluetooth();

	//This is an attempt at an I2C hard reset of all devices.
	//Float SDA high, and toggle SCL through nine complete cycles at 100kHz (or slower). Then issue a STOP sequence
	/*GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,0x80);	//SDA High
	SysCtlDelay(I2C1_BIT_DELAY);
	int i = 0;
	for(i=0; i<9; i++){
		GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0x40); 	//SCL High
		SysCtlDelay(I2C1_BIT_DELAY);
		GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0x00);	//SCL Low
		SysCtlDelay(I2C1_BIT_DELAY);
	}
	GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0x00);	//SCL Low
	SysCtlDelay(I2C1_BIT_DELAY);
	GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,0x00);	//SDA Low
	SysCtlDelay(I2C1_BIT_DELAY);

	GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0x40);	//SCL High
	SysCtlDelay(I2C1_BIT_DELAY);
	GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,0x80);	//SDA High
	SysCtlDelay(I2C1_BIT_DELAY);*/


	//Serious credit to the man who made the Arduino version of this. he gave me addresses and equations. Sadly Arduino obfuscates what really is happening
	//Link posted on blog page
	//gyro address = 0x68 not 0x69
	/*GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);	//Set GPA7 as SDA

	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);	//Set GPA6 as SCL

	I2CMasterInitExpClk(I2C1_MASTER_BASE,SysCtlClockGet(),false);	//I think it operates at 100kbps
	I2CMasterEnable(I2C1_MASTER_BASE);
	//Initalize the accelerometer			Address = 0x53
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x02);
	UARTSend(0xAB);
	I2CTransmit(0x53,0x2D,0x00);
	I2CTransmit(0x53,0x2D,0x10);
	I2CTransmit(0x53,0x2D,0x08);

	//Initalize the gyroscope				Address = 0x68
	I2CTransmit(0x68,0x3E,0x00);
	I2CTransmit(0x68,0x15,0x07);
	I2CTransmit(0x68,0x16,0x1E);
	I2CTransmit(0x68,0x17,0x00);
	UARTSend(0xAC);*/
	//SysCtlReset();		//This fixes the I2C but it's way to slow to be
}

/**********************************************************************************************
 *								UART Functions
 *********************************************************************************************/
//Send a single char over UART if multiple chars, it is the calle's responsibility to handle including burst send settings
inline void UARTSend(char trans){
	while(UARTBusy(UART2_BASE));
	UARTCharPutNonBlocking(UART2_BASE,trans);
}

/**********************************************************************************************
 *								UART interrupt Handler
 *********************************************************************************************/
//UART Int Handler. Will handle the data recieved and put in an array. WILL NOT UNDERSTAND JUST STORE
void Uart2IntHandler(void){
	unsigned long ulStatus;

	ulStatus=UARTIntStatus(UART2_BASE, true);	//This reports if the interrupt was a transmit recieve etc, only reports one's setup to be detected in initalization could possibly remove transmit detection

	UARTIntClear(UART2_BASE, ulStatus);	//Clears the interrupt so it does not detect itself.

	if(ulStatus & UART_INT_TX){
		//Transmit was requested I don't think anything needs to be done here I could probably get rid of this and the interrupt for it but for now im leaving it. UARTSend does this job in a cleaner way
	}else if(ulStatus & UART_INT_RX || ulStatus & UART_INT_RT){	//If recieved data
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x08);
		while(UARTCharsAvail(UART2_BASE)){	//While there is still data available to read
			char buffer = UARTCharGetNonBlocking(UART2_BASE);	//Read the data into a buffer for scanning
			if((buffer == 0xFA)&&(commandAddress == -2)){	//Is it the first dummy byte
				commandAddress=-1;	//Set the command Address to -1 this way the dummy byte is the only starting byte for a packet to be accepeted
			}else if((buffer == 0xEB)&&(commandAddress == -1)){	//Is the second dummy byte read
				commandAddress=0;	//Prepare to read data the dummy byte's have been validated
			}else if(commandAddress>=0){						//Read in because it's not a dummy byte, prep for reading
				recievedCommands[commandAddress]=buffer;
				commandAddress++;	//Some efficiency could be done here. Remove this line than change the bottom to have ++commandAddress. But that's nitpicky stuff
				commandAddress = (commandAddress>11) ? -2 : commandAddress;			//If greater than 6 set to -1 else set to self
			}
		}
		UARTCount = 0; //Again not sure of datatype sizes so go with what works right.
	}
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x02);
}

/**********************************************************************************************
 *								I2C Functions
 *********************************************************************************************/
//This will setup the com for a device, declare the regiester to write to and then write to that regiester
//Note I2C waits for a response bit before continuing. If there is no device on the other end the masterbusy loop will NEVER exit
void I2CTransmit(unsigned char device, unsigned char regiester, unsigned char value){
	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device, false);	//Set Device to transmit to

	I2CMasterDataPut(I2C1_MASTER_BASE,regiester);	//Put on regiester to prep writting
	I2CMasterControl(I2C1_MASTER_BASE,I2C_MASTER_CMD_BURST_SEND_START);	//Send start bit and the first thing
	while(I2CMasterBusy(I2C1_MASTER_BASE));	//Wait till data sent

	I2CMasterDataPut(I2C1_MASTER_BASE,value);	//Put more data on
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);	//Send data and finish bit
	while(I2CMasterBusy(I2C1_MASTER_BASE));	//Wait till done


}

//Read from device using address and put the read values into buff. num=num of bytes to read
void I2CRead(unsigned char device, unsigned char regiester, unsigned char num, unsigned long *buff){
	//All commented code could likely be delted currently saving for record / emergency
	/*I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device, false);	//Set Device to transmit to

	I2CMasterDataPut(I2C1_MASTER_BASE,regiester);	//Put on regiester to prep writting
	I2CMasterControl(I2C1_MASTER_BASE,I2C_MASTER_CMD_SINGLE_SEND);
	//while(I2CMasterBusBusy(I2C1_MASTER_BASE));	//Wait till data sent
	while(I2CMasterBusy(I2C1_MASTER_BASE));		//this is good

	short i=0;
	for (i=0; i<num; i++){
		I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device, true);	//Set device to RECIEVE FROM
		/*I2CMasterDataPut(I2C1_MASTER_BASE,regiester+i);	//Put on regiester to prep writting
		I2CMasterControl(I2C1_MASTER_BASE,I2C_MASTER_CMD_SINGLE_SEND);
		//while(I2CMasterBusBusy(I2C1_MASTER_BASE));	//Wait till data sent
		while(I2CMasterBusy(I2C1_MASTER_BASE));
	 */
	/*if(i==0){
			I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		}else if(i==(num-1)){
			I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		}else{
			I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		}
		//while(I2CMasterIntStatus(I2C1_MASTER_BASE, false) == 0);
		while(I2CMasterBusy(I2C1_MASTER_BASE));
		buff[i] = I2CMasterDataGet(I2C1_MASTER_BASE);
		//I2CMasterIntClear(I2C1_MASTER_BASE);
	}
	//buff[0]=0x02;
	 */
	short i=0;	//Initalize i because CCS doesn't like initalizing inside for loop :P
	for(i=0; i<num; i++){
		I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device, false);	//Set device to write to
		I2CMasterDataPut(I2C1_MASTER_BASE,regiester+i);	//Put on regiester to prep writting
		I2CMasterControl(I2C1_MASTER_BASE,I2C_MASTER_CMD_SINGLE_SEND);	//Make a single send there is no bursting through
		while(I2CMasterBusy(I2C1_MASTER_BASE));		//wait for the transmission to end via checking the master's state NOT THE BUS

		I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device, true);	//Set device to RECIEVE FROM
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);	//Set a single read because it will only return this address
		while(I2CMasterBusy(I2C1_MASTER_BASE));		//Wait for the master to stop recieving data

		buff[i] = I2CMasterDataGet(I2C1_MASTER_BASE);	//Place the data recieved (that is in the FIFO) into the buffer to return
	}

}

//Convert ACC readings into g force values
void rawAccToG(unsigned long *raw, float *RwAcc){
	float R;
	/*RwAcc[0]=raw[0];
	RwAcc[0]/=256;
	RwAcc[1]=0.01;*/
	//RwAcc[0]=((raw[1]<<8)|raw[0])/256;	//Again for some reason stacking commands is a no no. I can set RwAcc to 2 than divide but together it yields 0
	//RwAcc[1]=((raw[3]<<8)|raw[2])/256;
	//RwAcc[2]=((raw[5]<<8)|raw[4])/256;
	/*(RwAcc[0]=(float)(((raw[1])<<8)|raw[0]);	//Packet the two bytes into 1 and then divide by 256 because of how it's packeted
	RwAcc[0]/=256;		//Can be deleted. But isn't yet
	RwAcc[1]=(float)(((raw[3])<<8)|raw[2]);
	RwAcc[1]/=256;
	RwAcc[2]=(float)(((raw[5])<<8)|raw[4]);
	RwAcc[2]/=256;*/

	/*unsigned long test[2];
	test[0] = 180;
	test[1] = 10;
	short bob[1];
	bob[0] = ((char)test[0]<<8)|(char)test[1];
	float jimmy = (short)(((char)test[0]<<8)|(char)test[1]);
	jimmy /= 26;*/

	RwAcc[0] = (short)(((char)raw[1]<<8)|(char)raw[0]);
	RwAcc[1] = (short)(((char)raw[3]<<8)|(char)raw[2]);
	RwAcc[2] = (short)(((char)raw[5]<<8)|(char)raw[4]);

	R = floatingSquare(RwAcc[0]*RwAcc[0]+RwAcc[1]*RwAcc[1]+RwAcc[2]*RwAcc[2]);	//Find the magnitude of the vector

	RwAcc[0]/=R;		//Transform into unit vectors
	RwAcc[1]/=R;
	RwAcc[2]/=R;

	/*RwAcc[0]-=0.026;
	RwAcc[1]-=0.025;
	RwAcc[2]+=0.0006;*/

}

//Convert Gyro readings to Deg/sec
void rawGyroToDegsec(unsigned long *raw, float *Gyro_ds){
	/*Gyro_ds[0]=(((raw[2]<<8)|raw[3])-g_offx)/14.375;	//Im going to NEED to modify this like i did in the function above. making a float at once causes a 0 idk why
	Gyro_ds[1]=(((raw[4]<<8)|raw[5])-g_offy)/14.375;
	Gyro_ds[2]=(((raw[6]<<8)|raw[7])-g_offz)/14.375;
	Gyro_ds[3]=((raw[0]<<8)|raw[1]);
	 */
	//The offsets are found by holding the sensor steady and waiting to see if the values drift, if they do change the offset so it doesn't
	/*Gyro_ds[0]=(raw[2]<<8)|raw[3];	//So the compiler should make this more efficient BUT in debug I noticed making this stuff into floats on one line causes issues.
	Gyro_ds[0]-=g_offx;
	Gyro_ds[0] /= (float)14.375;
	Gyro_ds[1]=(raw[4]<<8)|raw[5];
	Gyro_ds[1]-=g_offy;
	Gyro_ds[1]/=(float)14.375;
	Gyro_ds[2]=(raw[6]<<8)|raw[7];
	Gyro_ds[2]-=g_offz;
	Gyro_ds[2]/=(float)14.375;
	Gyro_ds[3]=((raw[0]<<8)|raw[1]);*/
	Gyro_ds[0] = (short)(((char)raw[2]<<8)|(char)raw[3]);
	Gyro_ds[0]-=g_offx;
	Gyro_ds[0] /= (float)14.375;
	Gyro_ds[1] = (short)(((char)raw[4]<<8)|(char)raw[5]);
	Gyro_ds[1]-=g_offy;
	Gyro_ds[1]/=(float)14.375;
	Gyro_ds[2] = (short)(((char)raw[6]<<8)|(char)raw[7]);
	Gyro_ds[2]-=g_offz;
	Gyro_ds[2]/=(float)14.375;
	Gyro_ds[3] = (short)(((char)raw[0]<<8)|(char)raw[1]);
}
//Converts all the gyro acc data into usable data for roll pitch yaw... eventually
void getInclination(float *RwAcc, float *RwEst, float *RwGyro, float *Gyro_ds, float *Awz){
	//Long convaluted equation that someone found and i'm borrowing. Returns a unit vector need to convet to degrees
	float previousRwEst[3];
	previousRwEst[0]=RwEst[0];
	previousRwEst[1]=RwEst[1];
	previousRwEst[2]=RwEst[2];
	int w = 0;
	//int x = 0;
	float tmpf = 0.0;
	long currentTime, signRzGyro;
	float R;
	//currentTime = ((SysCtlClockGet() / (1000 * 3))*timeBetweenCalculations)-(SysTickValueGet()*(1000*3));	//So I BELIEVE this will tell me how many ms
	currentTime = (SysTickPeriodGet()-SysTickValueGet())/80000;	//I think this actually converts to ms between calculations
	interval = (currentTime - lastTime);	//
	interval = (interval < 0) ? interval + SysTickValueGet()/80000 : interval;
	lastTime = currentTime;
	//We need to fill in an inital RwEst for the math to work. We'll assume that the object is stable on the first start thus accelerometer angles are the angles
	//However, if we needed to restart mid flight this would fix itself fairly quickly in theory
	if (firstSample) { // the NaN check is used to wait for good data from the Stellaris
		for(w=0;w<=2;w++) {
			RwEst[w] = RwAcc[w];    //initialize with accelerometer readings
			previousRwEst[w] = RwEst[w];
		}
	}else{
		/*float bob = 0.7;			//This was here to debug float compares. The below RwEst statment is bugged!
		if(bob < 0.1){
			UARTSend(0xD4);
		}*/
		//Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
		//in this case skip the gyro data and just use previous estimate
		//My comment on this, The RzEST is very small which means if me divide by a small number we will get close to infinity which is bad
		if((fabs(RwEst[2])) < ((float)0.1)) {
			for(w=0;w<=2;w++) {
				RwGyro[w] = RwEst[w];
			}
		}else{
			//get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
			//This calculates the Axz Ayz part. It seems right
			for(w=0;w<=1;w++){
				tmpf = Gyro_ds[w];                        //get current gyro rate in deg/s
				tmpf *= interval / ((float)1000.0);                     //get angle change in deg
				Awz[w] = atan2f(RwEst[w],RwEst[2]) * 180.0 / PI;   //get angle and convert to degrees
				Awz[w] += tmpf;             //get updated angle according to gyro movement		//These are in degrees
			}
			//Awz[0] = 30;
			//Awz[1] = 45;
			//estimate sign of RzGyro by looking in what qudrant the angle Axz is,
			//RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
			signRzGyro = ( cosf(Awz[0] * PI / 180) >=0 ) ? 1 : -1;
			//reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
			//Effectively RwGyro is unitless. Completly unitless
			RwGyro[0] = sinf(Awz[0] * PI / 180);
			RwGyro[0] /= floatingSquare( 1 + (cosf(Awz[0] * PI / 180)*cosf(Awz[0] * PI / 180)) * (tanf(Awz[1] * PI / 180)*tanf(Awz[1] * PI / 180)));
			RwGyro[1] = sinf(Awz[1] * PI / 180);
			RwGyro[1] /= floatingSquare( 1 + (cosf(Awz[1] * PI / 180)*cosf(Awz[1] * PI / 180)) * (tanf(Awz[0] * PI / 180)*tanf(Awz[0] * PI / 180)));

			RwGyro[2] = signRzGyro * floatingSquare((float)1.0 - (RwGyro[0]*RwGyro[0]) - (RwGyro[1]*RwGyro[1]));	//THIS THIS IS THE DEVIL!		//Note this does not ever try to eval sqrt(<0)
			/*RwGyro[0] = 0.2;
			RwGyro[1] = 0.3;
			RwGyro[2] = 0.4;*/
		}
		//combine Accelerometer and gyro readings
		//THIS FOR LOOP CAUSES THE SYSTEM :P
		//w = 0;	failed attempt
		for(w=0;w<=2;w++){
			//I believe the units are in g's
			//RwEst[w] = (RwAcc[w] + (float)10.0 * RwGyro[w]) / (1 + 10);	failed attempt
			//RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);		failed attempt
			/*RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]);
			RwEst[w] /= (1+wGyro);		failed attempt */
			//RwEst[w] = nextafter(fmaf(wGyro,RwGyro[w],RwAcc[w]),100000000);		//So when I add RwAcc[2] + RwGyro[2] it crashes invebitably. WTF!!!!!
			RwEst[w] = floatingAdd(RwGyro[w],wGyro*RwAcc[w])/(1+wGyro);			//DO NOT REMOVE FLOATING ADD it for some reason allows this addition to happen which otherwise fails misserably.
		}
		/*for(x=0;x<=2;x++){		failed attempt
			RwEst[x] = (RwAcc[x] + wGyro * RwGyro[x]) / (1 + wGyro);
		}*/

		R = floatingSquare(RwEst[0]*RwEst[0]+RwEst[1]*RwEst[1]+RwEst[2]*RwEst[2]);

		RwEst[0]/=R;
		RwEst[1]/=R;	//This is unitless.
		RwEst[2]/=R;

		RwEst[0] = 0.9*RwEst[0] + 0.1*previousRwEst[0];
		RwEst[1] = 0.9*RwEst[1] + 0.1*previousRwEst[1];
		RwEst[2] = 0.9*RwEst[2] + 0.1*previousRwEst[2];

		R = floatingSquare(RwEst[0]*RwEst[0]+RwEst[1]*RwEst[1]+RwEst[2]*RwEst[2]);

		RwEst[0]/=R;
		RwEst[1]/=R;	//This is unitless.
		RwEst[2]/=R;
	}

	firstSample = 0;
}


/**********************************************************************************************
 *								SysTick Functions
 *********************************************************************************************/
void SysTickIntHandler(void){
	/*//UARTSend(0xAA);
	//Note going for float to char does not work, well it actually may work. It seems that the debugger will show char's as well characters so trying to use it as an int8 is ok but hard to debug. REALLY NEED TO FIND COMPILER'S DATA TYPE SIZES
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x08); //RBG is the order 2 4 8. display green when updating
	float quadPitch = 0;
	float quadRoll = 0;
	short androidPitch = 0;
	short androidRoll = 0;
	float pitchOffset = 0;
	float rollOffset = 0;
	unsigned long temp = 0;

	//Handles if the Stellaris UART is no longer listening to the Xbee and such.Should handle it anyway. No promises.
	if(UARTCount == previousCount){
		SysCtlPeripheralReset(SYSCTL_PERIPH_UART2);
		UARTCount += 1;
	}

	/*I2CRead(0x53,0x32,6,quadAcc);			//Address blah blah 2 for each axis
	rawAccToG(quadAcc,RwAcc);
	//Get Gyro data
	//Gyro ITG-3200 I2C
	//registers:
	//temp MSB = 1B, temp LSB = 1C
	//x axis MSB = 1D, x axis LSB = 1E
	//y axis MSB = 1F, y axis LSB = 20
	//z axis MSB = 21, z axis LSB = 22`

	I2CRead(0x68,0x1B,8,quadGyro);			//Address blah blah 2 for each axis + 2 for temperature. why. because why not
	rawGyroToDegsec(quadGyro,Gyro_ds);

	//Get the actual angles in XYZ. Store them in RwEst
	getInclination(RwAcc, RwEst, RwGyro, Gyro_ds, Awz);*/
	//After this function is called RwEst will hold the roll pitch and yaw
	//RwEst will be returned in YAW, PITCH, ROLL 0, 1, 2 remember this order very important. Little obvious but yaw is worthless

	//rotateX(HALF_PI * -RwEst[0]);
	//rotateZ(HALF_PI * RwEst[1]);
	//quadPitch = abs(atan2(RwEst[2],RwEst[0]) * 180 / PI);
	//quadRoll = abs(atan2(RwEst[2],RwEst[1]) * 180 / PI);
	/*quadPitch = fabs(RwEst[0]) * 90;
	quadRoll = fabs(RwEst[1]) * 90;*/

	/*recievedCommands[0] = 90;
	recievedCommands[1] = 90;
	recievedCommands[2] = 50;
	recievedCommands[3] = 0xA0;
	recievedCommands[4] = 2;*/


	//quadPitch = RwEst[0] * 90;
	//quadRoll = RwEst[1] * 90;

	/*quadPitch = 0;
	quadRoll = 0;

	//UARTSend(fabs(quadPitch));
	//UARTSend(fabs(quadRoll));
	//Note yaw should be tossed because it's garbage. just figuring out what yaw is.

	androidRoll = recievedCommands[0] - 90;
	androidPitch = recievedCommands[1] - 90;

	/*UARTSend(fabs(RwAcc[0]*100));
	UARTSend(fabs(RwAcc[1]*100));
	UARTSend(fabs(RwAcc[2]*100));
	UARTSend(fabs(Gyro_ds[0]));
	UARTSend(fabs(Gyro_ds[1]));
	UARTSend(fabs(Gyro_ds[2]));*/
	//UARTSend(atan2f((float).3,(float).6)*180/PI);

	/*pitchOffset = (androidPitch - quadPitch)/1;
	rollOffset = (quadRoll - androidRoll)/1;

	SysTickPeriodSet((SysCtlClockGet() / (1000 * 3))*timeBetweenCalculations);		//The time isn't set instantly but should be delayed till later in the code. Also this is not doing ms effects

	//char recievedPitchVal=0;					These are the chars in the packet order expected to be stored in the array
	//char recievedRollVal=0;
	//char recievedYawVal=0;
	//char recievedThrottle
	//char recievedSpecialCommand=0;
	//commandAddress = (commandAddress>4) ? -2 : commandAddress;		? syntax
	//PWMSet(TIMER0_BASE,recievedCommands[3] - pitchOffset);
	if(recievedCommands[4] == 2){		//If in manual mode		change it to a 2 i put it at 3 to disable it basically
		if(recievedCommands[3]!=0xCF){
			//Note 207 is off im trying 190 to make sure the motors don't bug out
			temp = ((recievedCommands[3] - pitchOffset + (recievedCommands[2]-50))>190) ? 190 : (floor(recievedCommands[3] - pitchOffset + (recievedCommands[2]-50)));
			temp = (temp < 2) ? 2 : (floor(temp));
			//UARTSend(temp);
			PWMSet(TIMER0_BASE,temp);

			temp = ((recievedCommands[3] + pitchOffset + (recievedCommands[2]-50))>190) ? 190 : (floor(recievedCommands[3] + pitchOffset + (recievedCommands[2]-50)));
			temp = (temp < 2) ? 2 : (floor(temp));
			//UARTSend(temp);
			PWMSet(TIMER1_BASE,temp);

			temp = ((recievedCommands[3] - rollOffset - (recievedCommands[2]-50))>190) ? 190 : (floor(recievedCommands[3] - rollOffset - (recievedCommands[2]-50)));
			temp = (temp < 2) ? 2 : (floor(temp));
			//UARTSend(temp);
			PWMSet(TIMER2_BASE,temp);

			temp = ((recievedCommands[3] + rollOffset - (recievedCommands[2]-50))>190) ? 190 : (floor(recievedCommands[3] + rollOffset - (recievedCommands[2]-50)));
			temp = (temp < 2) ? 2 : (floor(temp));
			//UARTSend(temp);
			PWMSet(TIMER3_BASE,temp);}
		else{
			PWMSet(TIMER0_BASE,0xCF);
			PWMSet(TIMER1_BASE,0xCF);
			PWMSet(TIMER2_BASE,0xCF);
			PWMSet(TIMER3_BASE,0xCF);
		}
	}else if(recievedCommands[4]==1){	//Hover mode
		//float temp = (raw[5]<<8)|(raw[4]);
		//temp/=256;
		//if(temp>1){

		//}

	}else if(recievedCommands[4]==0){
		PWMSet(TIMER0_BASE,0xCF);
		PWMSet(TIMER1_BASE,0xCF);
		PWMSet(TIMER2_BASE,0xCF);
		PWMSet(TIMER3_BASE,0xCF);
	}else if(recievedCommands[4]==15){
		PWMSet(TIMER0_BASE,recievedCommands[3]);
		PWMSet(TIMER1_BASE,recievedCommands[3]);
		PWMSet(TIMER2_BASE,recievedCommands[3]);
		PWMSet(TIMER3_BASE,recievedCommands[3]);
	}

	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x04);	//Display blue when flying
	//ARTSend(0x0D);	 */

	SysTickPeriodSet((SysCtlClockGet() / (1000 * 3))*timeBetweenCalculations);
}


/**********************************************************************************************
 *								Watchdog Functions
 *********************************************************************************************/
void WatchdogIntHandler(void){
	WatchdogIntClear(WATCHDOG_BASE);
	WatchdogReloadSet(WATCHDOG_BASE, 0xFEEFEEFF);	//Feed the dog a new time
	//SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
}

/**********************************************************************************************
 *								VoltageCutoff interrupt handler
 *********************************************************************************************/
void voltageCutoffHanlder(void){
	GPIOPinIntClear(GPIO_PORTE_BASE,GPIO_PIN_3);
	caseyIsPoop = 1;
}

/**********************************************************************************************
 *								Special Functions
 *********************************************************************************************/
//This will reset the bluetooth connection, during this time the Stellaris will be stuck using old data so flight is mildly dangerous.
//In future events I need to make the Stellaris go into hover mode when these things are happening.
void ResetBluetooth(void){
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0A);
	//The FIFO should be 16X8 (16 bytes, 8 bit wide each) so these 3 commands should not fill up the FIFO no need to wait
	UARTSend(0x24);	//Set to command mode
	UARTSend(0x24);
	UARTSend(0x24);	//When this is called a 0x4340440D0A will be thrown back at me which Uart2IntHandler will see

	//I could wait for the data to be sent to me but my interrupt handler isn't meant for that and also if the device didn't see this it definitely won't see this
	SysCtlDelay((SysCtlClockGet() / (1000 * 3))*100);	//Delay 100ms
	UARTSend(0x52);	//Send the reset command
	UARTSend(0x2C);
	UARTSend(0x31);
	UARTSend(0x0D);

	SysCtlDelay((SysCtlClockGet() / (1000 * 3))*100);	//Delay 100ms
}


//Floating point squareroot because Stellaris != like sqrt
float floatingSquare(float x){
	float y=1.0;
	while(fabs(x/y-y)>0.0001){
		y = (y+x/y)/2;
	}
	return y;
}
float floatingAdd(float x, float y){
	return x+y;
}
/**********************************************************************************************
 *								Main
 *********************************************************************************************/
void main(void) {

	float quadPitch = 0;
	float quadRoll = 0;
	short androidPitch = 0;
	short androidRoll = 0;
	float pitchOffset = 0;
	float rollOffset = 0;
	float final_pitchOffset = 0;
	float final_rollOffset = 0;
	long temp;

	recievedCommands[8] = 0xCF;
	recievedCommands[9] = 0xCF;
	recievedCommands[10] = 0xCF;
	recievedCommands[11] = 0xCF;

	//After tomorrow's test flight. FOR THE LOVE OF GOD MOVE THESE INITALIZATIONS TO FUNCTIONS
	/**********************************************************************************************
	 *								Local Variables
	 *********************************************************************************************/

	//unsigned long ultrasonic = 0;

	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	FPULazyStackingEnable();

	//Set the clock speed to 80MHz aka max speed
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	/*unsigned long test[2];
	test[0] = 180;
	test[1] = 10;
	short bob[1];
	bob[0] = ((char)test[0]<<8)|(char)test[1];
	float jimmy = (short)(((char)test[0]<<8)|(char)test[1]);
	jimmy /= 26;*/

	/**********************************************************************************************
	 *								Peripheral Initialization Awake
	 *********************************************************************************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	//Turn on GPIO communication on F pins for switches
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	//Turn on GPIO for Voltage cutoff
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	//Turn on GPIO for the PWM comms
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	//Turn on GPIO for LED test
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);   	//Turn on GPIO for UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	//Turn on Timer for PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);	//Turn on Timer for PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);	//Turn on Timer for PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);	//Turn on Timer for PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);		//Turn on I2C communication I2C slot 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);	//Turn on the UART com
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG);		//Turn on the watchdog timer. This is a risky idea but I think it's for the best.


	/**********************************************************************************************
	 *								Peripheral Initialization Sleep
	 *********************************************************************************************/
	/*SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);	//This sets what peripherals are still enabled in sleep mode while UART would be nice, it would require the clock operate at full speed which is :P
	SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_TIMER1);
	SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_TIMER2);
	SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_TIMER3);
	SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_I2C1);
	SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_ADC);
	SysCtlPeripheralClockGating(true); 		//I'm not sure about this one maybe remove it
	 */

	/**********************************************************************************************
	 *										PWM Initialization
	 *********************************************************************************************/
	SysCtlDelay((SysCtlClockGet() / (1000 * 3))*100);		//This shouldn't be needed will test to remove
	//PWM pin Setup
	//PWM 0 on GPIO PB6, PWM 1 on pin 4... etc
	GPIOPinConfigure(GPIO_PB6_T0CCP0);	//Pitch -		yaw +
	GPIOPinConfigure(GPIO_PB4_T1CCP0);	//Pitch +		yaw +
	GPIOPinConfigure(GPIO_PB0_T2CCP0);	//Roll -		yaw -
	GPIOPinConfigure(GPIO_PB2_T3CCP0);	//Roll +		yaw -
	GPIOPinTypeTimer(GPIO_PORTB_BASE, (GPIO_PIN_6|GPIO_PIN_4|GPIO_PIN_0|GPIO_PIN_2));

	//Prescale the timers so they are slow enough to work with the ESC
	TimerPrescaleSet(TIMER0_BASE,TIMER_A,2);
	TimerPrescaleSet(TIMER1_BASE,TIMER_A,2);
	TimerPrescaleSet(TIMER2_BASE,TIMER_A,2);
	TimerPrescaleSet(TIMER3_BASE,TIMER_A,2);

	//Basic LED Out Test	Not sure why this is here look into			This just turns on an LED that I don't have plugged in. should remove later
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,0xFF);

	//GPIOPinTypeGPIOOutputOD(GPIO_PORTB_BASE,GPIO_PIN_0);


	//Timers Setup for PWM and the load for the countdown
	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM);
	TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod -1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM);
	TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriod -1);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM);
	TimerLoadSet(TIMER2_BASE, TIMER_A, (ulPeriod -1));
	TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM);
	TimerLoadSet(TIMER3_BASE, TIMER_A, ulPeriod -1);

	//TimerPrescaleSet(TIMER2_BASE, TIMER_A, extender1);
	//TimerLoadSet(TIMER2_BASE, TIMER_A, period1);
	//Set the match which is when the thing will pull high
	TimerMatchSet(TIMER0_BASE, TIMER_A, 254);  //Duty cycle = (1-%desired)*1000 note this means this number is percent low not percent high
	TimerMatchSet(TIMER1_BASE, TIMER_A, 254);
	TimerMatchSet(TIMER2_BASE, TIMER_A, 254);
	TimerMatchSet(TIMER3_BASE, TIMER_A, 254);

	//TimerPrescaleMatchSet(TIMER2_BASE, TIMER_A, extender2);
	//TimerMatchSet(TIMER2_BASE, TIMER_A, period2);

	//Enable the timers
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);
	TimerEnable(TIMER2_BASE, TIMER_A);
	TimerEnable(TIMER3_BASE, TIMER_A);

	//SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1000000);

	PWMSet(TIMER0_BASE,998);
	PWMSet(TIMER1_BASE,998);
	PWMSet(TIMER2_BASE,998);
	PWMSet(TIMER3_BASE,998);

	/**********************************************************************************************
	 *									onboard	Chip interrupt Initialization
	 *********************************************************************************************/
	//These two buttons are used to reset the bluetooth module in case of disconnection
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	//RGB LED's
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00);
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;				//Sw1 (PF4) is unaviable unless you make it only a GPIOF input via these commands
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);	//Onboard buttons (PF0=Sw2,PF4=Sw1
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);		//This will make the buttons falling edge (a press pulls them low)
	//void (*functionPtr)(void) = &onBoardInteruptHandle;
	GPIOPortIntRegister(GPIO_PORTF_BASE, onBoardInteruptHandle);	//set function to handle interupt
	GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_FALLING_EDGE);		//Set the interrupt as falling edge
	GPIOPinIntEnable(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);	//Enable the interrupt
	//IntMasterEnable();
	IntEnable(INT_GPIOF);

	/**********************************************************************************************
	 *								UART Initialization
	 *********************************************************************************************/
	//Unlock PD7
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

	GPIOPinConfigure(GPIO_PD7_U2TX);			//Set PD7 as TX
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_7);

	GPIOPinConfigure(GPIO_PD6_U2RX);			//Set PD6 as RX
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6);

	UARTConfigSetExpClk(UART2_BASE,SysCtlClockGet(),115200,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);  //I believe the Xbee defaults to no parity I do know it's 9600 baud though, changed to 115200 for bluetooth reasons

	UARTFIFOLevelSet(UART2_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);	//Set's how big the fifo needs to be in order to call the interrupt handler, 2byte
	UARTIntRegister(UART2_BASE,Uart2IntHandler);	//Regiester the interrupt handler
	UARTIntClear(UART2_BASE, UART_INT_TX | UART_INT_RX);	//Clear the interrupt
	UARTIntEnable(UART2_BASE, UART_INT_RX);	//Enable the interrupt to trigger on both TX and RX event's. Could possibly remove TX
	UARTIntDisable(UART2_BASE,UART_INT_TX);

	UARTEnable(UART2_BASE);	//Enable UART
	IntEnable(INT_UART2);	//Second way to enable handler not sure if needed using anyway

	/**********************************************************************************************
	 *								I2C Initialization
	 *********************************************************************************************/
	//Serious credit to the man who made the Arduino version of this. he gave me addresses and equations. Sadly Arduino obfuscates what really is happening
	//Link posted on blog page
	//gyro address = 0x68 not 0x69
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);	//Set GPA7 as SDA

	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);	//Set GPA6 as SCL

	I2CMasterInitExpClk(I2C1_MASTER_BASE,SysCtlClockGet(),false);	//I think it operates at 100kbps
	I2CMasterEnable(I2C1_MASTER_BASE);
	//Initalize the accelerometer			Address = 0x53
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x02);
	//UARTSend(0xAB);
	I2CTransmit(0x53,0x2D,0x00);
	I2CTransmit(0x53,0x2D,0x10);
	I2CTransmit(0x53,0x2D,0x08);

	//Initalize the gyroscope				Address = 0x68
	I2CTransmit(0x68,0x3E,0x00);
	I2CTransmit(0x68,0x15,0x07);
	I2CTransmit(0x68,0x16,0x1E);
	I2CTransmit(0x68,0x17,0x00);
	//UARTSend(0xAC);

	/**********************************************************************************************
	 *								SysTick Initialization
	 *********************************************************************************************/
	SysTickIntRegister(SysTickIntHandler);
	SysTickIntEnable();

	SysTickPeriodSet((SysCtlClockGet() / (1000 * 3))*timeBetweenCalculations);	//This sets the period for the delay. the last num is the num of milliseconds
	SysTickEnable();

	/**********************************************************************************************
	 *								Watchdog Initialization
	 *********************************************************************************************/
	WatchdogReloadSet(WATCHDOG_BASE, 0xFEEFEEFF);	//Set the timer for a reset
	WatchdogIntRegister(WATCHDOG_BASE,WatchdogIntHandler);			//Enable interrupt
	WatchdogIntClear(WATCHDOG_BASE);
	WatchdogIntEnable(WATCHDOG_BASE);
	WatchdogEnable(WATCHDOG_BASE);	//Enable the actual timer
	IntEnable(INT_WATCHDOG);

	/**********************************************************************************************
	 *								Voltage cutoff Initialization
	 *********************************************************************************************/
	/*GPIOPinTypeGPIOInput(GPIO_PORTE_BASE,GPIO_PIN_3);
	GPIOPortIntRegister(GPIO_PORTE_BASE, voltageCutoffHanlder);
	GPIOIntTypeSet(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_RISING_EDGE);		//Set the interrupt as rising edge
	GPIOPinIntEnable(GPIO_PORTE_BASE,GPIO_PIN_3);	//Enable the interrupt
	//IntMasterEnable();
	IntEnable(INT_GPIOF);*/

	/**********************************************************************************************
	 *								Preflight motor inialization maybe not necessary not going to test
	 *********************************************************************************************/

	PWMSet(TIMER0_BASE,998);
	PWMSet(TIMER1_BASE,998);
	PWMSet(TIMER2_BASE,998);
	PWMSet(TIMER3_BASE,998);
	recievedCommands[0]=253;
	SysCtlDelay((SysCtlClockGet() / (1000 * 3))*100);	//Very important to ensure motor see's a start high (998 makes 0 sense but it works so shhhh don't tell anyone)
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x06);

	while(1){
		/*if((RwAcc[0]==0)&&(RwAcc[1]==0)&&(RwAcc[2]==0)){
			SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
			SysCtlDelay(10);
		}*/

		WatchdogReloadSet(WATCHDOG_BASE, 0xFEEFEEFF);	//Feed the dog a new time

		//UARTSend(recievedCommands[0]);
		//SysCtlDelay(50000);
		//Set 4 PWM Outputs

		//Get Acc data
		I2CRead(0x53,0x32,6,quadAcc);			//Address blah blah 2 for each axis
		rawAccToG(quadAcc,RwAcc);
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x04);		//Blue

		//Get Gyro data
		/**************************************
		  Gyro ITG-3200 I2C
		  registers:
		  temp MSB = 1B, temp LSB = 1C
		  x axis MSB = 1D, x axis LSB = 1E
		  y axis MSB = 1F, y axis LSB = 20
		  z axis MSB = 21, z axis LSB = 22
		 *************************************/
		I2CRead(0x68,0x1B,8,quadGyro);			//Address blah blah 2 for each axis + 2 for temperature. why. because why not
		rawGyroToDegsec(quadGyro,Gyro_ds);
		//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x02);		//Red

		//Get the actual angles in XYZ. Store them in RwEst
		getInclination(RwAcc, RwEst, RwGyro, Gyro_ds, Awz);

		//Remove later maybe this is systick
		//
		//UARTSend(0xAA);
		//Note going for float to char does not work, well it actually may work. It seems that the debugger will show char's as well characters so trying to use it as an int8 is ok but hard to debug. REALLY NEED TO FIND COMPILER'S DATA TYPE SIZES
		/*if(caseyIsPoop){
			//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0E);
		}else{
			//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x04); //RBG is the order 2 4 8. display green when updating
		}*/

		//Handles if the Stellaris UART is no longer listening to the Xbee and such.Should handle it anyway. No promises.
		/*if(UARTCount > 2500 ){
			SysCtlPeripheralReset(SYSCTL_PERIPH_UART2);
			UARTCount = 0;
			recievedCommands[3] = 0xCF;
		}
		UARTCount += 1;*/

		quadPitch = RwEst[0] * 90;
		quadRoll = RwEst[1] * 90;

		//quadPitch = 0;
		//quadRoll = 0;

		//UARTSend(fabs(quadPitch));
		//UARTSend(fabs(quadRoll));
		//Note yaw should be tossed because it's garbage. just figuring out what yaw is.

		//androidPitch = recievedCommands[0] - 90;
		//androidRoll = recievedCommands[1] - 90;
		androidPitch = 0;
		androidRoll = 0;
		/*UARTSend(fabs(RwAcc[0]*100));
			UARTSend(fabs(RwAcc[1]*100));
			UARTSend(fabs(RwAcc[2]*100));
			UARTSend(fabs(Gyro_ds[0]));
			UARTSend(fabs(Gyro_ds[1]));
			UARTSend(fabs(Gyro_ds[2]));*/
		//UARTSend(atan2f((float).3,(float).6)*180/PI);

		pitchOffset = (androidPitch - quadPitch)/1.0;
		rollOffset = (androidRoll - quadRoll)/1.0;

		//PID Stuff below
		/*float previous_pitch_error = pitch_error;
		float previous_roll_error = roll_error;
*/
		pitch_error = pitchOffset;
		roll_error = rollOffset;

		/*integral_pitch += pitch_error*interval/1000;
		integral_roll += roll_error*interval/1000;*/
		/*integral_pitch = 0;
		integral_roll = 0;*/

		derivative_pitch = Gyro_ds[1];
		derivative_roll = Gyro_ds[0];
		float kp_pitch = ((recievedCommands[5])/100.0);
		//float kd_pitch = 0.15;
		float kp_roll = ((recievedCommands[5])/100.0);
		//float kd_roll = 0.15;
		final_pitchOffset = ((kp_pitch*pitch_error)+(((recievedCommands[7])/100.0)*derivative_pitch));
		final_rollOffset = ((kp_roll*roll_error)-(((recievedCommands[7])/100.0)*derivative_roll));
		//char recievedPitchVal=0;					These are the chars in the packet order expected to be stored in the array
		//char recievedRollVal=0;
		//char recievedYawVal=0;
		//char recievedThrottle
		//char recievedSpecialCommand=0;
		//commandAddress = (commandAddress>4) ? -2 : commandAddress;		? syntax
		//PWMSet(TIMER0_BASE,recievedCommands[3] - pitchOffset);
		temp = 0xAA;
		if(recievedCommands[4] == 2){		//If in manual mode		change it to a 2 i put it at 3 to disable it basically
			if(recievedCommands[3]!=0xCF){
				//Note 207 is off im trying 190 to make sure the motors don't bug out
				//temp = ((recievedCommands[3] - final_pitchOffset + (recievedCommands[2]-50))>190) ? 190 : (floor(recievedCommands[3] - final_pitchOffset + (recievedCommands[2]-50)));
				//temp = (temp < 2) ? 2 : (floor(temp));
				temp = ((recievedCommands[9] - final_pitchOffset + (recievedCommands[2]-50))<2) ? 2 : (floor(recievedCommands[9] - final_pitchOffset + (recievedCommands[2]-50)));
				temp = (temp > 190) ? 190 : (floor(temp));
				//UARTSend(temp);
				PWMSet(TIMER0_BASE,temp);

				/*temp = ((recievedCommands[3] + final_pitchOffset + (recievedCommands[2]-50))>190) ? 190 : (floor(recievedCommands[3] + final_pitchOffset + (recievedCommands[2]-50)));
				temp = (temp < 2) ? 2 : (floor(temp));*/
				temp = ((recievedCommands[11] + final_pitchOffset + (recievedCommands[2]-50))<2) ? 2 : (floor(recievedCommands[11] + final_pitchOffset + (recievedCommands[2]-50)));
				temp = (temp > 190) ? 190 : (floor(temp));
				//UARTSend(temp);
				PWMSet(TIMER1_BASE,temp);

				/*temp = ((recievedCommands[3] - final_rollOffset - (recievedCommands[2]-50))>190) ? 190 : (floor(recievedCommands[3] - final_rollOffset - (recievedCommands[2]-50)));
				temp = (temp < 2) ? 2 : (floor(temp));*/
				temp = ((recievedCommands[8] - final_rollOffset - (recievedCommands[2]-50))<2) ? 2 : (floor(recievedCommands[8] - final_rollOffset - (recievedCommands[2]-50)));
				temp = (temp > 190) ? 190 : (floor(temp));
				//UARTSend(temp);
				PWMSet(TIMER2_BASE,temp);

				/*temp = ((recievedCommands[3] + final_rollOffset - (recievedCommands[2]-50))>190) ? 190 : (floor(recievedCommands[3] + final_rollOffset - (recievedCommands[2]-50)));
				temp = (temp < 2) ? 2 : (floor(temp));*/
				temp = ((recievedCommands[10] + final_rollOffset - (recievedCommands[2]-50))<2) ? 2 : (floor(recievedCommands[10] + final_rollOffset - (recievedCommands[2]-50)));
				temp = (temp > 190) ? 190 : (floor(temp));
				//UARTSend(temp);
				PWMSet(TIMER3_BASE,temp);
			}else{
				PWMSet(TIMER0_BASE,0xCF);
				PWMSet(TIMER1_BASE,0xCF);
				PWMSet(TIMER2_BASE,0xCF);
				PWMSet(TIMER3_BASE,0xCF);
			}
		}else if(recievedCommands[4]==1){	//Hover mode
			//float temp = (raw[5]<<8)|(raw[4]);
			//temp/=256;
			//if(temp>1){

			//}

		}else if(recievedCommands[4]==0){
			PWMSet(TIMER0_BASE,0xCF);
			PWMSet(TIMER1_BASE,0xCF);
			PWMSet(TIMER2_BASE,0xCF);
			PWMSet(TIMER3_BASE,0xCF);
		}else if(recievedCommands[4]==15){
			PWMSet(TIMER0_BASE,recievedCommands[3]);
			PWMSet(TIMER1_BASE,recievedCommands[3]);
			PWMSet(TIMER2_BASE,recievedCommands[3]);
			PWMSet(TIMER3_BASE,recievedCommands[3]);
		}

		/*if(caseyIsPoop){
			//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0E);
		}else{
			//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x08); //RBG is the order 2 4 8. display green when updating
		}*/
		//UARTSend(0x0D);

		//Remove later systick


		//After this function is called RwEst will hold the roll pitch and yaw
		//RwEst will be returned in PITCH, ROLL, YAW 0, 1, 2 remember this order very important. Little obvious but yaw is worthless

		/*if(RwEst[1]>0.5){
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x06);		//Red Blue, Correct data read in
		}else{
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0A);		//Red Green, The correct data is not there
		}*/
		/*GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x06);		//Red Blue, Correct data read in
		float test=RwAcc[0]*100;		//These two commands work
		char temp = (char)test;
		//UARTSend((char)(RwAcc[0])*100);	//This one does not
		UARTSend(temp);
		//UARTSend((char)(RwAcc[1])*100);

		UARTSend(0xAA);
		SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1);
		 */
	}

}
