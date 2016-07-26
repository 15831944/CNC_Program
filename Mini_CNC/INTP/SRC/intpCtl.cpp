#include "stdafx.h"
#include "..\INC\intp.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "..\..\LINK\INC\link.h"
#include "..\..\cnc\inc\cnc.h"

//// preload function
extern int  intpMain(INTP_DATA   *intpDataPtr);
extern int  intpReset(INTP_DATA  *intpDataPtr);

extern CNC_DATA *cncDataPtr;
extern LINK_DATA *linkDataPtr;
extern int  initSlope(INTP_DATA    *intpDataPtr);
extern SLOPE_DATA slopeData;
// ********************************************************************************************
// INTPInit()
// ********************************************************************************************
int intpInit(INTP_DATA *intpDataPtr)
{
	int i = 0;
  intpDataPtr ->IntpState = INTP_IDLE;
  linkDataPtr->intpToMmi.blockValid = false;
	intpDataPtr->TimeInterval = linkDataPtr->timeInterval;

	for(i = 0 ; i < 3 ; i++)
	{
		if( i == 0 )
		{
			intpDataPtr->XYZ_Start[i] = 150;
			intpDataPtr->XYZ_End[i] = 150;
			intpDataPtr->XYZNew[i] = 150;
			intpDataPtr->XYZOld[i] = 150;
			intpDataPtr->UVW_Start[i] = 0;
			intpDataPtr->UVWNew[i] = 0;
			intpDataPtr->UVW_End[i] = 0;
			intpDataPtr->UVWOld[i] = 0;
			intpDataPtr->nowPosition[i] = 150;
			linkDataPtr->intpToMmi.xyzPos[i] = intpDataPtr->XYZ_Start[i];
			linkDataPtr->intpToMmi.uvwPos[i] = intpDataPtr->UVW_Start[i];
		}

		else
		{
			intpDataPtr->XYZ_Start[i] = 0;
			intpDataPtr->XYZ_End[i] = 0;
			intpDataPtr->XYZNew[i] = 0;
			intpDataPtr->XYZOld[i] = 0;
			intpDataPtr->UVW_Start[i] = 0;
			intpDataPtr->UVWNew[i] = 0;
			intpDataPtr->UVW_End[i] = 0;
			intpDataPtr->UVWOld[i] = 0;
			intpDataPtr->nowPosition[i] = 0;
      linkDataPtr->intpToMmi.xyzPos[i]= intpDataPtr->XYZ_Start[i];
		  linkDataPtr->intpToMmi.uvwPos[i] = intpDataPtr->UVW_Start[i];
		}
		intpDataPtr->nowUVW[i] = 0.0;
	}

	intpDataPtr->IntpCtlState = INTP_CTL_RUNNING;

	intpDataPtr->Speed = 0.0;

	intpDataPtr->Working = true;

	initSlope(intpDataPtr);


	return 1;
}

// ********************************************************************************************
// INTPHandler()
// ********************************************************************************************
int intpCtl (INTP_DATA *intpDataPtr)
{
	// get cmd from sys , STOP, RESET from link port
	if((linkDataPtr->sysToIntp.sysToIntpCmd != SYS_TO_INTP_NULL) && (intpDataPtr->IntpCtlState == INTP_CTL_RUNNING))
	{
		switch (linkDataPtr->sysToIntp.sysToIntpCmd)
		{
		case SYS_TO_INTP_RESET:
			intpDataPtr->IntpCtlState = INTP_CTL_RESET_START;
			break;

		case SYS_TO_INTP_STOP:
			intpDataPtr->IntpFeedHold = 1;
			break;
		}
	}

	switch(intpDataPtr->IntpCtlState)
	{
	case INTP_CTL_RUNNING:
        intpMain(intpDataPtr);
	    	break;

	case INTP_CTL_RESET_START:
				intpDataPtr->IntpFeedHold = 1;
				intpDataPtr->IntpCtlState = INTP_CTL_RESET_WAIT;
				intpMain(intpDataPtr); // EXPECT intpmain and slope will proccess this stop cmd
				break;

	case INTP_CTL_RESET_WAIT:
				if(intpDataPtr->slopeData.nowSpeed == 0)
	      		intpDataPtr->IntpCtlState = INTP_CTL_RESET_WAIT_GO;

				intpMain(intpDataPtr);
				break;

	case INTP_CTL_RESET_WAIT_GO:

				break;

	case INTP_CTL_RESET_STOP:
				intpReset(intpDataPtr);
				linkDataPtr->sysToIntp.sysToIntpCmd = SYS_TO_INTP_NULL; //inform sys Reset done
		// if by reset the write side of a linked data make the necessary reset reset the written data interface
		// then wait as a read side of a linked data, until the linked data is reset
				intpDataPtr->IntpCtlState = INTP_CTL_RESET_WAIT_DEC_FIFO;
		//or
		//intpDataPtr->IntpCtlState = INTP_CTL_RUNNING;
		// or let the sys to clr all linked data after all FBs clr its own internal data and each FB waits restart signal from SYS
				intpMain(intpDataPtr);
				break;

	case INTP_CTL_RESET_WAIT_DEC_FIFO:
		// if dectointpfifo empty or dec fifo reset ok , go
		// intpDataPTr->intpCltState = INTP_CTL_RUNNING
		// if dec send a reset ok signal , then  go
		// intpDataPTr->intpCltState = INTP_CTL_RUNNING
				break;

	case INTP_CTL_RESET_ACTION:
				break;
	}

	  return 1;
}

// ********************************************************************************************
// INTPClose()
// ********************************************************************************************
int intpClose(INTP_DATA *intpDataPtr)
{

    return 1;
}
