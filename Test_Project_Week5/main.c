/*
 * File created by Patrick Cross. Created 3/4/2013 Finished N/A
 *
 * This file while called test file will likely house all of the code for the quad copter just in unusable chunks. Not sure yet.
 * So far it can do UART Transmission and PWM output
 * Next on the list is UART in and I2C com
 */



/*
 * The other ultrasonic may be used. Distance = ((Duration of high level)*(Sonic :340m/s))/2
 * Things that need to be looked into / implemented
 * putting the device into sleep mode.
 * ADC readings
 * Something fun for the hell of it idk
 * The actual main program
 * The new and correct UART data handler. 0xFAEB
 * Auto landing feature
 * Hover mode
 * Lift off
 * Stream video using a GoPro
 */


/**********************************************************************************************
 *								Include files
 *********************************************************************************************/
#include <math.h>

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
short recievedCommands[7];			//For the same reason here I will only be reading chars in here BUT the char get function can return -1 which would overflow the char
short commandAddress=-2;			//Can not use byte they are unsigned by default
//short recievedRollVal=0;					These are the chars in the packet order expected to be stored in the array
//short recievedPitchVal=0;
//short recievedYawVal=0;
//char recievedSpecialCommand=0;

//Offsets for the gyroscope. CHIP SPECIFIC!
char g_offx = 66;
char g_offy = 41;
char g_offz = 12;

char firstSample = 1;
long lastTime = 0;
long interval = 0;

char timeBetweenCalculations=10;		//Time in ms
float PI=3.1415;
float wGyro = 10.0;

/**********************************************************************************************
 *								Prototyping
 *********************************************************************************************/
//To be done soon

/**********************************************************************************************
 *								PWM Functions
 *********************************************************************************************/
//This function will set the PWM for the desired timer and thus desired pin on off for duty cycle from 0-998 note OFF not ON.
//We do not need to disable and enable the timer before calling timer match set it will just update the value
inline void PWMSet(unsigned long timer, unsigned long dutyLow){
	//TimerPrescaleMatchSet(timer,TIMER_A,(dutyLow << 8));
	TimerMatchSet(timer,TIMER_A,(dutyLow << 8));
	SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1);
	unsigned long bob = TimerMatchGet(timer,TIMER_A);
	unsigned long jim = TimerLoadGet(timer,TIMER_A);
	/*unsigned long pwm_period = SysCtlClockGet() / 1000000*dutyLow;
	int pwm_extender = pwm_period >> 16;
	pwm_period &= 0xFFFF;

    TimerPrescaleMatchSet(TIMER0_BASE, TIMER_A, pwm_extender);
    TimerMatchSet(TIMER0_BASE, TIMER_A, pwm_period);*/
}

/**********************************************************************************************
 *								ADC Functions
 *********************************************************************************************/
//Return the value on ADC still in construction
unsigned long ADCRead(){
	unsigned long ulValue;
	ADCProcessorTrigger(ADC_BASE, 0);
	while(!ADCIntStatus(ADC_BASE, 0, false))
	{
	}
	ADCSequenceDataGet(ADC_BASE, 0, &ulValue);
	return ulValue;
}


/**********************************************************************************************
 *								On Board Interrupt
 *********************************************************************************************/
void onBoardInteruptHandle(void){
	long tmp = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);
	if(tmp==0x00){		//Both buttons pressed
		if(asleep){
			//SysCtlReset();
		}else{
			asleep=1;			//Right now it doesn't actually stay asleep, I'm not sure what is happening really it could be resetting to quickly or just never really sleep
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00);		//Blue
			//SysCtlSleep();
		}
	}else if(tmp==0x01){		//Sw1 pressed
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x02);		//Red
	}else if(tmp==0x10){		//Sw2 pressed
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x04);		//Blue
	}else{						//Nothing pressed
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x08);		//Green
	}
	//GPIOPinIntClear(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);		//This turns off the interrupt do not uncomment, I may be wrong about that statment
}

/**********************************************************************************************
 *								UART Functions
 *********************************************************************************************/
//Send a single char over UART
inline void UARTSend(char trans){
	UARTCharPutNonBlocking(UART2_BASE,trans);
}

/**********************************************************************************************
 *								UART interrupt Handler
 *********************************************************************************************/
//UART Int Handler. Will handle the data recieved and put in an array. WILL NOT UNDERSTAND JUST STORE
void Uart2IntHandler(void){
	unsigned long ulStatus;

	ulStatus=UARTIntStatus(UART2_BASE, true);

	UARTIntClear(UART2_BASE, ulStatus);

	if(ulStatus & UART_INT_TX){
		//Transmit was requested I don't think anything needs to be done here I could probably get rid of this and the interrupt for it but for now im leaving it. UARTSend does this job in a cleaner way
	}else if(ulStatus & UART_INT_RX || ulStatus & UART_INT_RT){
		while(UARTCharsAvail(UART2_BASE)){
			char buffer = UARTCharGetNonBlocking(UART2_BASE);
			if((buffer == 0xFA)&&(commandAddress == -2)){
				commandAddress=-1;	//Set the command Address to 0 this way the dummy byte is the only starting byte for a packet to be accepeted
			}else if((buffer == 0xEB)&&(commandAddress == -1)){
				commandAddress=0;
			}else if(commandAddress>=0){						//Read in because it's not a dummy byte, prep for reading
				recievedCommands[commandAddress]=buffer;
				commandAddress++;	//Some efficiency could be done here. Remove this line than change the bottom to have ++commandAddress. But that's nitpicky stuff
				commandAddress = (commandAddress>6) ? -2 : commandAddress;			//If greater than 6 set to -1 else set to self
			}
		}
	}

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
	short i=0;
	for(i=0; i<num; i++){
		I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device, false);	//Set device to write to
		I2CMasterDataPut(I2C1_MASTER_BASE,regiester+i);	//Put on regiester to prep writting
		I2CMasterControl(I2C1_MASTER_BASE,I2C_MASTER_CMD_SINGLE_SEND);
		while(I2CMasterBusy(I2C1_MASTER_BASE));		//this is good

		I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device, true);	//Set device to RECIEVE FROM
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
		while(I2CMasterBusy(I2C1_MASTER_BASE));

		buff[i] = I2CMasterDataGet(I2C1_MASTER_BASE);
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
	RwAcc[0]=(raw[1]<<8)|(raw[0]);
	RwAcc[0]/=256;
	RwAcc[1]=(raw[3]<<8)|(raw[2]);
	RwAcc[1]/=256;
	RwAcc[2]=(raw[5]<<8)|(raw[4]);
	RwAcc[2]/=256;

	R = sqrt(RwAcc[0]*RwAcc[0]+RwAcc[1]*RwAcc[1]+RwAcc[2]*RwAcc[2]);

	RwAcc[0]/=R;
	RwAcc[1]/=R;
	RwAcc[2]/=R;
}

//Convert Gyro readings to Deg/sec
void rawGyroToDegsec(unsigned long *raw, float *Gyro_ds){
	/*Gyro_ds[0]=(((raw[2]<<8)|raw[3])-g_offx)/14.375;	//Im going to NEED to modify this like i did in the function above. making a float at once causes a 0 idk why
	Gyro_ds[1]=(((raw[4]<<8)|raw[5])-g_offy)/14.375;
	Gyro_ds[2]=(((raw[6]<<8)|raw[7])-g_offz)/14.375;
	Gyro_ds[3]=((raw[0]<<8)|raw[1]);
	*/
	Gyro_ds[0]=(raw[2]<<8)|raw[3];	//So the compiler should make this more efficient BUT in debug I noticed making this stuff into floats on one line causes issues.
	Gyro_ds[0]-=g_offx;
	Gyro_ds[0]/=14.375;
	Gyro_ds[1]=(raw[4]<<8)|raw[5];
	Gyro_ds[1]-=g_offx;
	Gyro_ds[1]/=14.375;
	Gyro_ds[2]=(raw[6]<<8)|raw[7];
	Gyro_ds[2]-=g_offx;
	Gyro_ds[2]/=14.375;
	Gyro_ds[3]=((raw[0]<<8)|raw[1]);
}
//Converts all the gyro acc data into usable data for roll pitch yaw... eventually
void getInclination(float *RwAcc, float *RwEst, float *RwGyro, float *Gyro_ds, float *Awz){
	int w = 0;
	float tmpf = 0.0;
	long currentTime, signRzGyro;
	float R;
	currentTime = (SysTickValueGet()*(1000*3));	//So I BELIEVE this will tell me how many ms
	interval = (currentTime - lastTime)+timeBetweenCalculations;	//So big factoid issue thing because this WILL be called by the main sys tick looping it will actually miss an entire cycle before calling thus need of delay
	lastTime = currentTime;
	if (firstSample) { // the NaN check is used to wait for good data from the Stellaris
		for(w=0;w<=2;w++) {
			RwEst[w] = RwAcc[w];    //initialize with accelerometer readings
		}
	}else{
		//Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
		//in this case skip the gyro data and just use previous estimate
		if(abs(RwEst[2]) < 0.1) {
			for(w=0;w<=2;w++) {
				RwGyro[w] = RwEst[w];
			}
		}else{
			//get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst

			for(w=0;w<=1;w++){
				tmpf = Gyro_ds[w];                        //get current gyro rate in deg/s
				tmpf *= interval / 1000.0f;                     //get angle change in deg
				Awz[w] = atan2(RwEst[w],RwEst[2]) * 180 / PI;   //get angle and convert to degrees
				Awz[w] += tmpf;             //get updated angle according to gyro movement
			}
			//estimate sign of RzGyro by looking in what qudrant the angle Axz is,
			//RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
			signRzGyro = ( cos(Awz[0] * PI / 180) >=0 ) ? 1 : -1;

			//reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
			for(w=0;w<=1;w++){
				RwGyro[0] = sin(Awz[0] * PI / 180);
				RwGyro[0] /= sqrt( 1 + (cos(Awz[0] * PI / 180)*cos(Awz[0] * PI / 180)) * (tan(Awz[1] * PI / 180)*tan(Awz[1] * PI / 180)));
				RwGyro[1] = sin(Awz[1] * PI / 180);
				RwGyro[1] /= sqrt( 1 + (cos(Awz[1] * PI / 180)*cos(Awz[1] * PI / 180)) * (tan(Awz[0] * PI / 180)*tan(Awz[0] * PI / 180)) );
			}
			RwGyro[2] = signRzGyro * sqrt(1 - (RwGyro[0]*RwGyro[0]) - (RwGyro[1]*RwGyro[1]));

		}
		//combine Accelerometer and gyro readings
		for(w=0;w<=2;w++){
			RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);
		}

		R = sqrt(RwEst[0]*RwEst[0]+RwEst[1]*RwEst[1]+RwEst[2]*RwEst[2]);

		RwEst[0]/=R;
		RwEst[1]/=R;
		RwEst[2]/=R;
	}

	firstSample = false;
}


/**********************************************************************************************
 *								SysTick Functions
 *********************************************************************************************/
void SysTickIntHandler(void){
	float quadPitch = 0;
	float quadRoll = 0;
	/*So here will be the basically timer interrupt handler kind of deal. It's meant to
	SysTickPeriodSet((SysCtlClockGet() / (1000 * 3))*timeBetweenCalculations);
	I2CRead(0x53,0x32,6,quadAcc);			//Address blah blah 2 for each axis
	rawAccToG(quadAcc,RwAcc);
	//Get Gyro data
		  //Gyro ITG-3200 I2C
		  //registers:
		  //temp MSB = 1B, temp LSB = 1C
		  //x axis MSB = 1D, x axis LSB = 1E
		  //y axis MSB = 1F, y axis LSB = 20
		  //z axis MSB = 21, z axis LSB = 22

	I2CRead(0x68,0x1B,8,quadGyro);			//Address blah blah 2 for each axis + 2 for temperature. why. because why not
	rawGyroToDegsec(quadGyro,Gyro_ds);

	//Get the actual angles in XYZ. Store them in RwEst
	getInclination(RwAcc, RwEst, RwGyro, Gyro_ds, Awz);
	//After this function is called RwEst will hold the roll pitch and yaw
	//rotateX(HALF_PI * -RwEst[0]);
   	//rotateZ(HALF_PI * RwEst[1]);
	quadPitch = PI/2 * -RwEst[0];
	quadRoll = PI/2 * RwEst[1];
	*/
}


/**********************************************************************************************
 *								Watchdog Functions
 *********************************************************************************************/
void WatchdogIntHandler(void){
	WatchdogIntClear(WATCHDOG_BASE);
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
}

/**********************************************************************************************
 *								Special Functions
 *********************************************************************************************/


/**********************************************************************************************
 *								Main
 *********************************************************************************************/
void main(void) {

	/**********************************************************************************************
	 *								Local Variables
	 *********************************************************************************************/
	unsigned long quadAcc[6];
	unsigned long quadGyro[8];
	float RwAcc[3];
	float Gyro_ds[3];
	float RwGyro[3];
	float Awz[2];
	float RwEst[3];

	unsigned long ultrasonic = 0;

	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	FPULazyStackingEnable();

	//Set the clock speed to 80MHz aka max speed
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	//SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

	/**********************************************************************************************
	 *								Peripheral Initialization Awake
	 *********************************************************************************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	//Turn on GPIO communication on F pins for switches
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	//Turn on GPIO for ADC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	//Turn on GPIO for the PWM comms
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	//Turn on GPIO for LED test
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);   	//Turn on GPIO for UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	//Turn on Timer for PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);	//Turn on Timer for PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);	//Turn on Timer for PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);	//Turn on Timer for PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);		//Turn on I2C communication I2C slot 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);		//Turn on the ADC slot. Need to look up pins
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
	SysCtlDelay((SysCtlClockGet() / (1000 * 3))*5000);
	//PWM pin Setup
	//PWM 0 on GPIO PB6, PWM 1 on pin 4... etc
	GPIOPinConfigure(GPIO_PB6_T0CCP0);
	GPIOPinConfigure(GPIO_PB4_T1CCP0);
	GPIOPinConfigure(GPIO_PB0_T2CCP0);
	GPIOPinConfigure(GPIO_PB2_T3CCP0);
	GPIOPinTypeTimer(GPIO_PORTB_BASE, (GPIO_PIN_6|GPIO_PIN_4|GPIO_PIN_0|GPIO_PIN_2));

	TimerPrescaleSet(TIMER2_BASE,TIMER_A,2);
	//unsigned long period1 = SysCtlClockGet() / 1000; /*Hz*/
	/*int extender1 = period1 >> 16;
	period1 &= 0xFFFF;

	   //set default
	unsigned long period2 = 0;
	int extender2 = period2 >> 16;
	period2 &= 0xFFFF;*/

	//Basic LED Out Test	Not sure why this is here look into			This just turns on an LED that I don't have plugged in. should remove later
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,0xFF);

	//GPIOPinTypeGPIOOutputOD(GPIO_PORTB_BASE,GPIO_PIN_0);


	//Timers Setup for PWM
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

	TimerMatchSet(TIMER0_BASE, TIMER_A, 998);  //Duty cycle = (1-%desired)*1000 note this means this number is percent low not percent high
	TimerMatchSet(TIMER1_BASE, TIMER_A, 998);
	TimerMatchSet(TIMER2_BASE, TIMER_A, 254);
	TimerMatchSet(TIMER3_BASE, TIMER_A, 998);

    //TimerPrescaleMatchSet(TIMER2_BASE, TIMER_A, extender2);
    //TimerMatchSet(TIMER2_BASE, TIMER_A, period2);

	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);
	TimerEnable(TIMER2_BASE, TIMER_A);
	TimerEnable(TIMER3_BASE, TIMER_A);


	/**********************************************************************************************
	 *										ADC Initialization
	 *********************************************************************************************/

	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);
	ADCSequenceConfigure(ADC_BASE,0,ADC_TRIGGER_PROCESSOR,0);
	ADCSequenceStepConfigure(ADC_BASE,0,ADC_TRIGGER_PROCESSOR,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
	ADCSequenceEnable(ADC_BASE,0);
	//Still need to change the refrence voltage to use a 5V refrence which I don't have yet.


	/**********************************************************************************************
	 *									onboard	Chip interrupt Initialization
	 *********************************************************************************************/
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	//RGB LED's
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00);
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;				//Sw1 (PF4) is unaviable unless you make it only a GPIOF input via these commands
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);	//Onboard buttons (PF0=Sw2,PF4=Sw1
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);		//This will make the buttons falling edge (a press pulls them low)
	/*void (*functionPtr)(void) = &onBoardInteruptHandle;
	GPIOPortIntRegister(GPIO_PORTF_BASE, onBoardInteruptHandle);	//set function to handle interupt
	GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_BOTH_EDGES);		//Set the interrupt as rising edge
	GPIOPinIntEnable(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);	//Enable the interrupt
	 */
	/*
	IntMasterEnable();
	IntEnable(INT_GPIOF);
	 */

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

	UARTFIFOLevelSet(UART2_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);
	UARTIntRegister(UART2_BASE,Uart2IntHandler);
	UARTIntClear(UART2_BASE, UART_INT_TX | UART_INT_RX);
	UARTIntEnable(UART2_BASE, UART_INT_TX | UART_INT_RX);

	UARTEnable(UART2_BASE);
	IntEnable(INT_UART2);

	/**********************************************************************************************
	 *								I2C Initialization
	 *********************************************************************************************/
	//Serious credit to the man who made the Arduino version of this. Without him I would be reading data pages for the next 2 weeks :)
	//Link posted on blog page
	//gyro address = 0x68 not 0x69
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);	//Set GPA7 as SDA

	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);	//Set GPA6 as SCL

	I2CMasterInitExpClk(I2C1_MASTER_BASE,SysCtlClockGet(),false);	//I think it operates at 100kbps
	I2CMasterEnable(I2C1_MASTER_BASE);
	//Initalize the accelerometer			Address = 0x53
	/*I2CTransmit(0x53,0x2D,0x00);
	I2CTransmit(0x53,0x2D,0x10);
	I2CTransmit(0x53,0x2D,0x08);

	//Initalize the gyroscope				Address = 0x68
	I2CTransmit(0x68,0x3E,0x00);
	I2CTransmit(0x68,0x15,0x07);
	I2CTransmit(0x68,0x16,0x1E);
	I2CTransmit(0x68,0x17,0x00);
*/

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

	PWMSet(TIMER2_BASE,998);
	SysCtlDelay((SysCtlClockGet() / (1000 * 3))*5000);

	while(1){

		//This right here is for Zach to be able to test the Motors
		/*short i=0;
		for (i = 0; i<254; i++){
			PWMSet(TIMER2_BASE,254-i);
			SysCtlDelay((SysCtlClockGet() / (1000 * 3))*20);	//Delay 1ms
		}
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x04);
		for (i = 0; i<254; i++){
			PWMSet(TIMER2_BASE,i);
			SysCtlDelay((SysCtlClockGet() / (1000 * 3))*20);	//Delay 1ms
		}*/
		PWMSet(TIMER2_BASE,20);
		SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1000);
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x06);
		WatchdogReloadSet(WATCHDOG_BASE, 0xFEEFEEFF);	//Feed the dog a new time
		//Yea so there is some parts here that need to be uncommented nothing to big
		/*WatchdogReloadSet(WATCHDOG_BASE, 0xFEEFEEFF);	//Feed the dog a new time

		//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x06);
		//SysCtlDelay(50000);
		//Set 4 PWM Outputs
		PWMSet(TIMER0_BASE,500);			//The last number represents the amount of time low out of 1000
		PWMSet(TIMER1_BASE,750);
		PWMSet(TIMER2_BASE,250);
		PWMSet(TIMER3_BASE,0);
		ADCRead();

		// This section debugs the UART
		/*
		if(commandAddress==-1){			//If waiting for more data
			if(recievedCommands[1]==0xC8){
				GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x06);		//Red Blue, Correct data read in
			}else{
				GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0A);		//Red Green, The correct data is not there
			}
		}else{
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0C);			//Blue Green, Reading data currently
		}
		 */

		//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0A);		//Red Green, The correct data is not there
		//SysCtlDelay((SysCtlClockGet() / (1000 * 3))*5);
		//Get Acc data
		/*I2CRead(0x53,0x32,6,quadAcc);			//Address blah blah 2 for each axis
		rawAccToG(quadAcc,RwAcc);
		/*GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x04);		//Blue

		//Get Gyro data
		/**************************************
		  Gyro ITG-3200 I2C
		  registers:
		  temp MSB = 1B, temp LSB = 1C
		  x axis MSB = 1D, x axis LSB = 1E
		  y axis MSB = 1F, y axis LSB = 20
		  z axis MSB = 21, z axis LSB = 22
		 *************************************/
		/*I2CRead(0x68,0x1B,8,quadGyro);			//Address blah blah 2 for each axis + 2 for temperature. why. because why not
		rawGyroToDegsec(quadGyro,Gyro_ds);
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x02);		//Red

		//Get the actual angles in XYZ. Store them in RwEst
		getInclination(RwAcc, RwEst, RwGyro, Gyro_ds, Awz);
		//After this function is called RwEst will hold the roll pitch and yaw
		//RwEst will be returned in YAW, PITCH, ROLL 0, 1, 2 remember this order very important. Little obvious but yaw is worthless

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
		/*UARTSend(0xC8);
		SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1000);		//delay of 1ms * the last number
		UARTSend(0xC8);
		SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1000);		//delay of 1ms * the last number
		UARTSend(0xC8);
		SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1000);		//delay of 1ms * the last number
		UARTSend(0xC8);
		SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1000);		//delay of 1ms * the last number
		UARTSend(0xC8);
		SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1000);		//delay of 1ms * the last number
		UARTSend(0xC8);
		SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1000);		//delay of 1ms * the last number
		UARTSend(0xC8);
		SysCtlDelay((SysCtlClockGet() / (1000 * 3))*1000);		//delay of 1ms * the last number
		 */
	}

}
