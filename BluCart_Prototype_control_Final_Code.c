/**
	Project:	BluCart
	Programmer: O'Dale Cotterell
	Course:		ECET 493L - Senior Project II wLab
	Date:		10/14/2016
	Version:	3.0
	MCU:		HCS12 MC9S12G128
	IDE: 		CodeWarrior 
	------------
	Description:
	------------
	
	This code is developed for the control of the autonomous BluCart robotic car prototype. 
	Control of the prototype will focus on the main functionality of the cart, track & follow,
	obstacle avoidance, manual touch control and motor control.
	
**/

//includes
#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */


//declarations
unsigned char irBeaconInput, ultrasonicInput, touchSensorInput;

//motor functions
void initPWM(void);
void motor_left(void);
void motor_right(void);
void motor_forward(void);
void motor_stop(void);

//read sensor inputs
void readSensors(void);

//obstacle avoidance
void avoid(void);

//IR Beacon driver
void trackBeacon(void);

//other functions
void MSDelay(unsigned int);
void initRTI(void);
void initPorts(void);

/******************
	Main program
******************/
void main()
{	
	//system initialization
	initPorts();
	initPWM();
	readSensors();
	initRTI();
	
	
	CPMUCOP_CR = 0; // disable COP
	EnableInterrupts;
	
	//main loop
	for(;;)
	{
		//continuously read sensor values and track beacon 
		readSensors();
		trackBeacon();
	}

}

//function definitions

//motor functions
/*********************************************************
*	Configure PWM channel 0 and 1 to control motor speed
*********************************************************/
void initPWM()
{
	PWMCLK = 0x03;
	PWMSCLA = 2;
	PWMPOL = 0x03;
	PWMPER0 = 200;
	PWMPER1 = 200;
	PWMDTY0 = 175;
	PWMDTY1 = 175;
	//PWME = 0x03; // Turn both motors on.
}

/****************************************************
	Turn left motor ON.
	Stops the left motor and moves the right motor.
	PWM channel 1 (Left Motor) OFF 
	PWM channel 0 (Right Motor) ON
****************************************************/
void motor_left(void)
{
	PWME = 0x01;
}

/***************************************************
	Turn right motor ON.
	Stops the right motor and moves the left motor.
	PWM channel 1 (Left Motor) ON 
	PWM channel 0 (Right Motor) OFF
***************************************************/
void motor_right(void)
{
	PWME = 0x02;
}

/************************************
	Turn left & right motors ON.
	PWM channel 1 (Left Motor) ON 
	PWM channel 0 (Right Motor) ON
*************************************/
void motor_forward(void)
{
	PWME = 0x03;
}

/**************************************************
	Turn left & right motors OFF.
	Stops the left motor and stop the right motor.
	PWM channel 1 (Left Motor) OFF 
	PWM channel 0 (Right Motor) OFF
**************************************************/
void motor_stop(void)
{
	PWME = 0x00;
}

/***************************************
	Reads in the values from PORTA.
	PORTA7:4 is IR Beacon(WSEN) 
	PORTA3:1 is Ultrasonic (WNE) 
	PORTA0 is Touch Sensor
***************************************/
void readSensors(void)
{
	//read and parse values
	//Mask PORTA3:0 and shift PORTA7:4 values to bit 0 position
	irBeaconInput = (PORTA & 0xF0) >> 4;
	PTT = (PORTA & 0xF0);	//test code to view status of Beacon output
	//Mask all except PORTA3:1 and shift values to bit 0 position
	ultrasonicInput = (PORTA & 0x0E) >> 1;
	//Mask all except PORTA0 
	touchSensorInput = (PORTA & 0x01);
}

/******************************************
	Avoid obstacles based on sensor inputs
	LOW = Obstacle, HIGH = No Obstacle
******************************************/
void avoid(void)
{
		switch (ultrasonicInput)
		{
			//no obstacle detected, move forward
			case 7:
			{
				//let beacon decide
				break;
			}
			//obstacle on the right
			case 6:
			{
				while(ultrasonicInput == 6)
				{
					motor_left();
					readSensors();
				}
				break;
			}
			//obstacle in front
			case 5:
			{
				//stop motors until obstacle is gone
				while(ultrasonicInput == 5)
				{
					motor_stop();
					readSensors();
				}
				break;
			}
			//obstacle front and right
			case 4:
			{
				break;
			}
			//obstacle on the left
			case 3:
			{
				while(ultrasonicInput == 3)
				{
					motor_right();
					readSensors();
				}
				break;
			}
			//obstacle on left and right
			case 2:
			{
				break;
			}
			//obstacle left and front 
			case 1:
			{
				break;
			}
			default:
			{
				break;
			}
		}
}

/********************************************
	Read IR Beacon inputs and actuate motors
	Input: High means no detection
		   Low means detection
	Layout:00001111 (0000WSEN) 
********************************************/
void trackBeacon(void)
{
	switch(irBeaconInput)
	{
		//beacon detected north, move motor forward
		case 14:
		{
			motor_forward();
			break;
		}
		//beacon detected east
		case 13:
		{
			motor_right();
			break;
		}
		//beacon detected west
		case 7:
		{
			motor_left();
			break;
		}
		//beacon detected south 1011
		case 11:
		{
			motor_stop();
			break;
		}
		//no beacon detection 
		case 15:
		{
			motor_stop();
			break;
		}
	}
}

/**********************************
	Initialize Real Time Interrupt 
**********************************/
void initRTI(void)
{
	CPMURTI = 0x11;			// set RTI clock divide to do 488.28 Hz
	//CPMURTI = 0x80 + 0x39;			// set RTI clock divide to do 10 Hz
	CPMUINT_RTIE = 1;		// enable RTI
}

/*************************************************
	Initialize PORTS
	Set Data Direction Registers for Input/Output
*************************************************/
void initPorts(void)
{
	DDRA = 0x00; //PORTA as input for sensor values
	DDRT = 0xF0; //PT7-4 LED output
	PTT = 0xFF;
}
/***********************************
	Milisecond delay function
	Input: int time in mili-seconds
***********************************/
void MSDelay(unsigned int itime)
{
	unsigned int i; unsigned int j;
	for(i=0;i<itime;i++)
		for(j=0;j<1000;j++);
}

/***********************
*interrupts definitions
***********************/

#pragma CODE_SEG NON_BANKED
//------ Real Time Interrupt ISR 
interrupt (((0x10000-Vrti)/2)-1) void RTI_ISR(void)
{
	//while touch sensor pressed stop motors
	while(touchSensorInput)
	{
	  motor_stop();
	  readSensors();
	}

	//check ultrasonic sensors for obstacles 
	avoid();

	CPMUFLG = CPMUFLG | CPMUFLG_RTIF_MASK; //clear interrupt flag	
}

#pragma CODE_SEG DEFAULT


