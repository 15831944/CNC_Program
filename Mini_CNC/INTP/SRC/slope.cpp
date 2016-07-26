#include "stdafx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "..\..\DEC\INC\dec.h"

#include "..\INC\intp.h"

#include "..\..\CNC\INC\cnc.h"


extern INTP_DATA *intpDataPtr;
//*************************************************************************************
// initSlope
//*************************************************************************************
int  initSlope(INTP_DATA *intpDataPtr)
{
     intpDataPtr->slopeData.Step = 0.2;  // 8G * 0.001s

     intpDataPtr->slopeData.slopeState = SPEED_UP;

	 intpDataPtr->slopeData.takeoutR = false;

	 intpDataPtr->slopeData.getCycle = intpDataPtr->TimeInterval;

	 intpDataPtr->slopeData.intpEnd  = false;

	 intpDataPtr->slopeData.tLength = 0.0;
	 intpDataPtr->slopeData.sLength = 0.0;
	 intpDataPtr->slopeData.rLength = 0.0;

	 intpDataPtr->slopeData.slowdown = 0.0;

	 intpDataPtr->slopeData.accCount= 0;
	 intpDataPtr->slopeData.decCount= 0;
	 intpDataPtr->slopeData.constCount= 0;

	 intpDataPtr->slopeData.AccGo= 0;
	 intpDataPtr->slopeData.DecGo= 0;
	 intpDataPtr->slopeData.ConstGo= 0;

	 intpDataPtr->slopeData.decL= 0.0;
	 intpDataPtr->slopeData.accL= 0.0;
	 intpDataPtr->slopeData.constL= 0.0;

	 // INTP
	 intpDataPtr->slopeData.nowSpeed= 0.0;

     return 1;

}
//*************************************************************************************
// speedup
//*************************************************************************************
int  Speed_Up(INTP_DATA *intpDataPtr)
{
	double RestLength;

	intpDataPtr->slopeData.AccGo++;
	intpDataPtr->slopeData.nowSpeed += intpDataPtr->slopeData.Step;

	intpDataPtr->slopeData.slowdown += intpDataPtr->slopeData.nowSpeed * intpDataPtr->slopeData.getCycle*0.001;
	if(intpDataPtr->slopeData.AccGo == intpDataPtr->slopeData.accCount)
	{
		if(intpDataPtr->slopeData.constCount == 0)
		   intpDataPtr->slopeData.slopeState = SPEED_DOWN;

		else
		   intpDataPtr->slopeData.slopeState = SPEED_CONST;
	}

	RestLength = intpDataPtr->slopeData.tLength - intpDataPtr->slopeData.slowdown;

	if( RestLength <= 0.0  )
	{intpDataPtr->slopeData.intpEnd = true;}

	return 1;
}
//*************************************************************************************
// slowdown
//*************************************************************************************
int Slow_Down(INTP_DATA *intpDataPtr)
{
	double RestLength;

	intpDataPtr->slopeData.DecGo++;
	intpDataPtr->slopeData.nowSpeed -= intpDataPtr->slopeData.Step;
	intpDataPtr->slopeData.slowdown += intpDataPtr->slopeData.nowSpeed * intpDataPtr->slopeData.getCycle * 0.001;

	double check = 0.0;
	check = intpDataPtr->slopeData.rLength - (intpDataPtr->slopeData.nowSpeed * intpDataPtr->slopeData.getCycle*0.001);
	if ( check > -0.5 && check < 0.5 && intpDataPtr->slopeData.takeoutR == false)
	{
		intpDataPtr->slopeData.DecGo--;
		intpDataPtr->slopeData.takeoutR = true;
		intpDataPtr->slopeData.nowSpeed += intpDataPtr->slopeData.Step;
	}


	if(intpDataPtr->slopeData.DecGo == intpDataPtr->slopeData.decCount)
	{
		intpDataPtr->slopeData.intpEnd = true;
	}



	return 1;
}
//*************************************************************************************
// constspeed
//*************************************************************************************
int Const_Speed(INTP_DATA *intpDataPtr)
{
    double RestLength;

	intpDataPtr->slopeData.ConstGo++;
	intpDataPtr->slopeData.slowdown += intpDataPtr->slopeData.nowSpeed * intpDataPtr->slopeData.getCycle * 0.001;
	RestLength = intpDataPtr->slopeData.tLength - intpDataPtr->slopeData.slowdown;


	if(intpDataPtr->slopeData.ConstGo == intpDataPtr->slopeData.constCount &&  RestLength > 0)
	{
		intpDataPtr->slopeData.slopeState = SPEED_DOWN;
	}

	if(RestLength <= 0)
	{
		intpDataPtr->slopeData.intpEnd = true;

	}


	return 1;
}

//*************************************************************************************
// slopemotion function
//*************************************************************************************

int slope(INTP_DATA *intpDataPtr)
{
	switch(intpDataPtr->slopeData.slopeState)
	{

	case SPEED_UP:
		Speed_Up(intpDataPtr);
		break;

	case SPEED_DOWN:
		Slow_Down(intpDataPtr);
		break;

	case SPEED_CONST:
		Const_Speed(intpDataPtr);
		break;

	}


	return 1;
}
