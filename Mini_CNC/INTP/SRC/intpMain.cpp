#include "stdafx.h"
#include "..\INC\intp.h"
#include "..\..\CNC\INC\cnc.h"
#include "..\..\LINK\INC\link.h"
#include "..\..\DEC\INC\dec.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
extern CNC_DATA  *cncDataPtr;
extern LINK_DATA *linkDataPtr;

extern INTP_DATA *intpDataPtr;

extern int  initSlope(INTP_DATA    *intpDataPtr);
extern int  slope(INTP_DATA *intpDataPtr);

void backTrans(INTP_DATA *intpDataPtr);
// ********************************************************************************************
// ReadFIFO
// ********************************************************************************************
int rdFIFO(LINK_DATA *linkDataPtr , INTP_DATA *intpDataPtr)
{
  if (linkDataPtr->BLOCK.counter >= 0 && linkDataPtr->motionCtl.singleBlockTrigger == true)
  {
    if(linkDataPtr->motionCtl.singleBlock)
		linkDataPtr->motionCtl.singleBlockTrigger = false;

	if(linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].blockValid)
	{
		switch(linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].blockCommand)
	   {
		case G00_RAPID:
		case G01_LINEAR:
			linkDataPtr->intpToMmi.programEnd = false;
			intpDataPtr ->Speed = linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].feedrate;
			intpDataPtr->IntpState = INTP_LIN_PRE;

			if(linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].relative)
			{

				for(int i = 0 ; i < 3 ; i++)
					{
                        if(linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].motionFunction.motion.endValid[i])
						intpDataPtr->XYZ_End[i] = intpDataPtr->XYZ_Start[i] + linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].motionFunction.motion.endPoint[i];
				    }
			}

			else
			{
				for(int i = 0 ; i < 3 ; i++)
				{
			  if(linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].motionFunction.motion.endValid[i])
				intpDataPtr->XYZ_End[i] =  linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].motionFunction.motion.endPoint[i];
				}
			}

			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].blockValid =false;
			linkDataPtr->BLOCK.counter--;
			linkDataPtr->BLOCK.rdIndex = (++linkDataPtr->BLOCK.rdIndex )% FIFO_BLOCK_MAX;
			return 1;
			
		case G02_CIR_CW:
		case G03_CIR_CCW:
			linkDataPtr->intpToMmi.programEnd = false;
			intpDataPtr->Speed = linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].feedrate;
            intpDataPtr->IntpState = INTP_CIR_PRE;

			for(int i = 0 ; i < 3 ; i++)
			intpDataPtr->centerPoint[i] = linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].motionFunction.motion.centerPoint[i];

			intpDataPtr->relAbs = linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].relative;

			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].blockValid =false;
			linkDataPtr->BLOCK.counter--;
			linkDataPtr->BLOCK.rdIndex = (++linkDataPtr->BLOCK.rdIndex) % FIFO_BLOCK_MAX;
			return 1;
			
		case G04_DWELL:
			linkDataPtr->intpToMmi.programEnd = false;
			intpDataPtr->dwellT = linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].motionFunction.function.dwellTime.value;
			if (intpDataPtr->dwellT > 0)
			{
				intpDataPtr->counter   = (int)floor(intpDataPtr->dwellT * 1000 / linkDataPtr->timeInterval );  
				intpDataPtr->IntpState = INTP_DWELL;
			}

			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].blockValid =false;
			linkDataPtr->BLOCK.counter--;
			linkDataPtr->BLOCK.rdIndex = (++linkDataPtr->BLOCK.rdIndex) % FIFO_BLOCK_MAX;
			return 1;
		
		case M30_END:
			linkDataPtr->intpToMmi.programEnd = true;
			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].blockValid =false;
			linkDataPtr->BLOCK.counter--;
			linkDataPtr->BLOCK.rdIndex = (++linkDataPtr->BLOCK.rdIndex) % FIFO_BLOCK_MAX;
			return 1;
			
		case M01_PAUSE:
			linkDataPtr->intpToMmi.programEnd = false;
			linkDataPtr->motionCtl.singleBlockTrigger = false;
			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.rdIndex].blockValid =false;
			linkDataPtr->BLOCK.counter--;
			linkDataPtr->BLOCK.rdIndex = (++linkDataPtr->BLOCK.rdIndex )% FIFO_BLOCK_MAX;
			return 1;
			

		default:
			return 0;


	   }

	}
  }
	return 0;
}

// ********************************************************************************************
// intpMain()
// ********************************************************************************************
int intpMain(INTP_DATA *intpDataPtr)
{
	switch (intpDataPtr->IntpState)
	{
	case INTP_IDLE:
         intp_IDLE(intpDataPtr);
		break;

	case INTP_LIN_PRE:
         intp_LINPRE(intpDataPtr);
		break;

	case INTP_LIN_EXE:
         intp_LINEXE(intpDataPtr);
		break;

	case INTP_CIR_PRE:
         intp_CIRPRE(intpDataPtr);
		break;

	case INTP_CIR_EXE:
         intp_CIREXE(intpDataPtr);
		break;


	case INTP_DWELL:       
		intp_DWELL(intpDataPtr);
		break;

	case INTP_LIN_CLOSE:
		intp_LINCLOSE(intpDataPtr);
		break;

	case INTP_CIR_CLOSE:
        intp_CIRCLOSE(intpDataPtr);
		break;

	default:

		break;



	}

	return 1;
}


// ********************************************************************************************
// intpReset()
// ********************************************************************************************
int intpReset(INTP_DATA *intpDataPtr)
{
    int i ;

	for(i = 0 ; i < 3 ; i++)
	{
		linkDataPtr->intpToMmi.xyzPos[i] = intpDataPtr->nowPosition[i];

	}

	intpDataPtr->IntpState = INTP_IDLE;
	initSlope(intpDataPtr);
	return 1;
}


int intp_IDLE(INTP_DATA *intpDataPtr)
{

	if(rdFIFO(linkDataPtr,intpDataPtr))
	{
    return 1;
	}


	return 1;
}
int intp_LINPRE(INTP_DATA *intpDataPtr)
{
	int i;



		for( i = 0 ; i < 3 ; i++)
		{
			intpDataPtr->XYZ_Rel[i] = intpDataPtr->XYZ_End[i] - intpDataPtr->XYZ_Start[i];
			intpDataPtr->totalL += intpDataPtr->XYZ_Rel[i] * intpDataPtr->XYZ_Rel[i];
		}
		intpDataPtr->totalL = sqrt( intpDataPtr->totalL );



  
	if (intpDataPtr->totalL < 0.00001)
	{
		intpDataPtr->IntpState = INTP_LIN_CLOSE;
		return 1;
	}
   

	for( i = 0 ; i < 3 ; i++)
	{
		intpDataPtr->intpFactor[i] = intpDataPtr->XYZ_Rel[i] / intpDataPtr->totalL;
	}

	// go into pre slope function
	intpDataPtr->slopeData.accCount = 0;
    intpDataPtr->slopeData.constCount = 0;
	intpDataPtr->slopeData.decCount = 0;



	intpDataPtr->slopeData.getCycle = intpDataPtr->TimeInterval;
	intpDataPtr->slopeData.slopeState = SPEED_UP;
	intpDataPtr->slopeData.intpEnd    = false;
	int temp = (int)(intpDataPtr->Speed / intpDataPtr->slopeData.Step);
	intpDataPtr->slopeData.accCount   = temp;
	if(intpDataPtr->slopeData.accCount <= 0)
	{
		
	 // over ride
	
	}
	else
	{

		 for (int j = 1 ; j <= intpDataPtr->slopeData.accCount ; j++)
		 {
         intpDataPtr->slopeData.accL += intpDataPtr->slopeData.accCount * intpDataPtr->slopeData.Step * intpDataPtr->slopeData.getCycle*0.001;
		 }
		 intpDataPtr->slopeData.decL =  intpDataPtr->slopeData.accL;
		 intpDataPtr->slopeData.decCount =  intpDataPtr->slopeData.accCount;

	}
	 intpDataPtr->slopeData.constCount = (int) (intpDataPtr->totalL - 2*  intpDataPtr->slopeData.accL) / (intpDataPtr->Speed*0.02);
	 intpDataPtr->slopeData.constL =  intpDataPtr->slopeData.constCount * intpDataPtr->Speed *0.02;

	 intpDataPtr->slopeData.rLength = intpDataPtr->totalL - 2* intpDataPtr->slopeData.accL -  intpDataPtr->slopeData.constL;

	 intpDataPtr->slopeData.AccGo = 0 ;
	 intpDataPtr->slopeData.ConstGo = 0;
	 intpDataPtr->slopeData.DecGo = 0 ;


	if(intpDataPtr->slopeData.accCount <= 0 )
		intpDataPtr->slopeData.slopeState = SPEED_CONST;

	else
		intpDataPtr->slopeData.slopeState = SPEED_UP;

	 intpDataPtr->slopeData.nowSpeed = 0;

	 intpDataPtr->slopeData.tLength  = intpDataPtr->totalL;

	// slope pre complete

	intpDataPtr->restL = intpDataPtr->totalL;


	intpDataPtr->IntpState = INTP_LIN_EXE;
	return 1;
}

int intp_LINEXE(INTP_DATA *intpDataPtr)
{
   int i ;
   if(linkDataPtr->motionCtl.feedHold != true )
   {

	   slope(intpDataPtr);
	   intpDataPtr->NSpeed  = intpDataPtr->slopeData.nowSpeed; // use slope function
	   linkDataPtr->intpToMmi.NowSpeed = intpDataPtr->NSpeed;
	  /* intpDataPtr->DLength = intpDataPtr->NSpeed * intpDataPtr->TimeInterval;
	   intpDataPtr->nowL   += intpDataPtr->DLength;*/

	  

	   for (i = 0 ; i < 3 ; i++)
	   {
		   linkDataPtr->intpToMmi.uvwOld[i] = intpDataPtr->nowUVW[i];
		   intpDataPtr->UVWOld[i] = intpDataPtr->nowUVW[i];
		   intpDataPtr->nowPosition[i] = intpDataPtr->XYZ_Start[i] + ( intpDataPtr->slopeData.slowdown * intpDataPtr->intpFactor[i]) ;

	   }
      
	   if ((intpDataPtr->nowPosition[0]*intpDataPtr->nowPosition[0] + intpDataPtr->nowPosition[1] * intpDataPtr->nowPosition[1]) > (MAX_LENGTH * MAX_LENGTH))
	   {		
		   for (i = 0 ; i < 3 ; i++)
		   {
			   intpDataPtr->nowPosition[i] = intpDataPtr->XYZOld[i];
			   intpDataPtr->XYZ_End[i] = intpDataPtr->nowPosition[i];
		   }

		   if ((linkDataPtr->formToSys.OpMode == MEM) || (linkDataPtr->formToSys.OpMode == MDI))
		   {
			   intpDataPtr->IntpState = INTP_IDLE;
			   return 1;
		   }
		   else
		   {
			   intpDataPtr->IntpState = INTP_IDLE;
			   return 0;
		   }
	   }	 

	   else
	   {
		   backTrans(intpDataPtr);
		   intpDataPtr->UVW_End[0] = intpDataPtr->nowUVW[0];
		   intpDataPtr->UVW_End[1] = intpDataPtr->nowUVW[1];
		   intpDataPtr->UVW_End[2] = intpDataPtr->nowUVW[2];


		   for(int l = 0 ; l < 3 ; l++)
		   {
			   linkDataPtr->intpToMmi.xyzPos[l] = intpDataPtr->nowPosition[l];
			   linkDataPtr->intpToMmi.uvwPos[l] = intpDataPtr->nowUVW[l];
			   
		   }
		       
	   }

	   // FKT BKT

	   if( intpDataPtr->slopeData.intpEnd )
		{

			intpDataPtr->IntpState = INTP_LIN_CLOSE;
		}

   }

   return 1;

}
int intp_LINCLOSE(INTP_DATA *intpDataPtr)
{
	int i;

	for( i = 0 ; i < 3 ; i++)
	{
		intpDataPtr->nowPosition[i] = intpDataPtr->XYZ_End[i];
	}
	backTrans(intpDataPtr);


	for( i = 0 ; i < 3 ; i++)
	{
		linkDataPtr->intpToMmi.xyzPos[i] = intpDataPtr->nowPosition[i];
		linkDataPtr->intpToMmi.uvwPos[i] = intpDataPtr->nowUVW[i];
		linkDataPtr->intpToMmi.uvwOld[i] = intpDataPtr->nowUVW[i];
	}


	for(i = 0 ; i < 3 ; i++)
		intpDataPtr->XYZ_Start[i] = intpDataPtr->nowPosition[i];

	

	initSlope(intpDataPtr);
	intpDataPtr->nowL = 0.0;
	linkDataPtr->intpToMmi.blockValid = true;
	intpDataPtr->Working = false;
    intpDataPtr->IntpState = INTP_IDLE;

	return 1;

}
int intp_CIRPRE(INTP_DATA *intpDataPtr)
{
	return 1;
}
int intp_CIREXE(INTP_DATA *intpDataPtr)
{
	return 1;
}
int intp_CIRCLOSE(INTP_DATA *intpDataPtr)
{
	return 1;
}
int intp_DWELL(INTP_DATA *intpDataPtr)
{
	intpDataPtr->counter--;

	if( intpDataPtr->counter <= 0)
	{
		intpDataPtr->IntpState = INTP_IDLE;
		intpDataPtr->Working = false;
	}

	return 1;

}


void backTrans(INTP_DATA *intpDataPtr)
{

	double X;	double Y;	double Z;
	double L1 = 75;
	double L2 = 75;
	double angle1 = 0;//ид1-2╕╤1
	double angle2 = 0;//ид1-2╕╤2
	double angle3 = 0;//ид1+2╕╤1
	double angle4 = 0;//ид1+2╕╤2
	double angle5 = 0;//ид1 13
	double angle6 = 0;//ид2	13
	double angle7 = 0;//ид1 14
	double angle8 = 0;//ид2 14
	double angle9 = 0;//ид1 23
	double angle10 = 0;//ид2 23
	double angle11 = 0;//ид1 24
	double angle12 = 0;//ид2 24
	X = intpDataPtr->nowPosition[0];		Y = intpDataPtr->nowPosition[1];		Z = intpDataPtr->nowPosition[2];
	
	angle1 = acos( ( X*X + Y*Y - L1*L1 - L2*L2 ) / ( 2*L1*L2 ) );
	if ( angle1 >= PI )
		angle1 -= 2*PI;
	else if( angle1 <= -PI )
		angle1 += 2*PI;

	angle2 = -angle1;//ид1-2╕╤2

	if ( angle2 >= PI )
		angle2 -= 2*PI;
	else if( angle2 <= -PI )
		angle2 += 2*PI;

	angle3 = 2 * atan( Y/X );//ид1+2╕╤1
	if ( angle3 >= PI )
		angle3 -= 2*PI;
	else if( angle3 <= -PI )
		angle3 += 2*PI;
	angle4 = PI + angle3;//ид1+2╕╤2
	if ( angle4 >= PI )
		angle4 -= 2*PI;
	else if( angle4 <= -PI )
		angle4 += 2*PI;
	angle5 = ( angle3 + angle1 )/2;//ид1 13
	angle6 = ( angle3 - angle1 )/2;//ид2	13

	angle7 = ( angle4 + angle1 )/2;//ид1 14
	angle8 = ( angle4 - angle1 )/2;//ид2 14

	angle9 = ( angle3 + angle2 )/2;//ид1 23
	angle10 = ( angle3 - angle2 )/2;//ид2 23

	angle11 = ( angle4 + angle2 )/2;//ид1 24
	angle12 = ( angle4 - angle2 )/2;//ид2 24


	double temp1;	double temp2;	double temp3;	double temp4;
	double temp5;	double temp6;	double temp7;	double temp8;

	double a;	double b;	double c;	double d;
	temp1 = (intpDataPtr->UVWOld[0] - angle5);
	temp2 = (intpDataPtr->UVWOld[1] - angle6);
	a = temp1 * temp1 + temp2 * temp2;

	temp3 = (intpDataPtr->UVWOld[0] - angle7);
	temp4 = (intpDataPtr->UVWOld[1] - angle8);
	b = temp3 * temp3 + temp4 * temp4;

	temp5 = (intpDataPtr->UVWOld[0] - angle9);
	temp6 = (intpDataPtr->UVWOld[1] - angle10);
	c = temp5 * temp5 + temp6 * temp6;

	temp7 = (intpDataPtr->UVWOld[0] - angle11);
	temp8 = (intpDataPtr->UVWOld[1] - angle12);
	d = temp7 * temp7 + temp8 * temp8;

	if ( a<b && a<c && a<d )
	{
		intpDataPtr->nowUVW[0] = angle5;
		intpDataPtr->nowUVW[1] = angle6;
		intpDataPtr->nowUVW[2] = intpDataPtr->nowPosition[2];
	} 
	else if( b<a && b<c &&b<d )
	{
		intpDataPtr->nowUVW[0] = angle7;
		intpDataPtr->nowUVW[1] = angle8;
		intpDataPtr->nowUVW[2] = intpDataPtr->nowPosition[2];
	}
	else if( c<=a && c<=b && c<=d )
	{
		intpDataPtr->nowUVW[0] = angle9;
		intpDataPtr->nowUVW[1] = angle10;
		intpDataPtr->nowUVW[2] = intpDataPtr->nowPosition[2];
	}
	else if( d<a && d<b && d<c )
	{
		intpDataPtr->nowUVW[0] = angle11;
		intpDataPtr->nowUVW[1] = angle12;
		intpDataPtr->nowUVW[2] = intpDataPtr->nowPosition[2];
	}


}