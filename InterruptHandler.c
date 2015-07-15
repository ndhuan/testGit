/*
 * InterruptHandler.c
 *
 *  Created on: Nov 10, 2014
 *      Author: AntoniTran
 */
#include "include.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"

extern PIDType PIDVelocity;






int32_t set = 0;
UARTType UART_Q;
int16_t MaxSpeed = 90;

void HBridgeEnable(void)
{
	GPIOPinWrite(ENABLE_PORT, ENABLE_PIN, 0xFF);
}

void HBridgeDisable(void)
{
	GPIOPinWrite(ENABLE_PORT, ENABLE_PIN, 0);
}

void Timer5ISR(void)
{
	static uint8_t NumSpdSet = 0;
	ROM_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
#ifdef USE_QEI
	//Get Velocity
	Speed = ROM_QEIVelocityGet(QEI0_BASE) * ROM_QEIDirectionGet(QEI0_BASE);
	//Get Position
	Position = ((int32_t)ROM_QEIPositionGet(QEI0_BASE));
#else
	Speed = Speedtemp;
	Speedtemp = 0;
#endif

	NumSpdSet++;

	if (NumSpdSet == PIDVerLoop)	//PID position
	{
		NumSpdSet = 0;
		if (PIDPosition.Enable)
		{
#ifdef PID_POSITION
			UARTPut_uint32(UART0_BASE, Position);
#endif
			//PIDPosCalc(Position, MaxSpeed);
			PIDCalc(&PIDPosition, Position, 90, SPEEDPID_CYCLE);
			//SetPWM(DEFAULT,(long)PIDPosition.PIDResult);
			SetPWM_usingTimer(TIMER0_BASE,DEFAULT, (long)PIDPosition.PIDResult );
		}
	}
}


void UARTIntHandler(void)
{
	uint32_t ui32Status;
	uint16_t temp;
	static uint16_t RemainBytes=0,cmdStatus;
	static uint8_t UARTBuf[50],BufIndex=0;
	// Get the interrupt status.
	ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
	// Clear the asserted interrupts.
	ROM_UARTIntClear(UART0_BASE, ui32Status);
	//
	while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
	{
		temp=UARTCharGetNonBlocking(UART0_BASE);
		if (RemainBytes==0)
		{
			switch (temp)
			{
			case CMD_SET_CONFIG:
				RemainBytes=12;
				cmdStatus=CMD_SET_CONFIG;
				break;
			case CMD_START:

				PIDVelocity.Enable=true;
				HBridgeEnable();
				cmdStatus=CMD_START;

				break;
			case CMD_STOP:
				HBridgeDisable();
				PIDVelocity.Enable=false;
				PIDPosition.Enable=false;
				cmdStatus=CMD_STOP;
				break;
			case CMD_SETPOINT:
				RemainBytes=4;
				cmdStatus=CMD_SETPOINT;
				break;
			}
		}
		else
		{
			UARTBuf[BufIndex]=temp;
			RemainBytes--;
			BufIndex++;
			if(RemainBytes==0)
			{
				BufIndex=0;
				//
				switch(cmdStatus)
				{
				case CMD_SET_CONFIG:
					PIDPosition.Kp=(UARTBuf[0]<<24|UARTBuf[1]<<16|UARTBuf[2]<<8|UARTBuf[3])*1.0/1000000000;
					PIDPosition.Ki=(UARTBuf[4]<<24|UARTBuf[5]<<16|UARTBuf[6]<<8|UARTBuf[7])*1.0/1000000000;
					PIDPosition.Kd=(UARTBuf[8]<<24|UARTBuf[9]<<16|UARTBuf[10]<<8|UARTBuf[11])*1.0/1000000000;
					break;
				case CMD_SETPOINT:
					PIDPositionSet((UARTBuf[0])<<24|(UARTBuf[1])<<16|(UARTBuf[2])<<8|UARTBuf[3]);
					break;
				}
			}
		}
	}
}
#ifndef USE_QEI
void EncoderISR(void)
{
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);
	if (HWREG(GPIO_PORTD_BASE + 0x0200))
	{
		Speedtemp--;
		Position--;
	}
	else
	{
		Speedtemp++;
		Position++;
	}
}
#endif
