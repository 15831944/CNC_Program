#include "stdafx.h"
#include "..\INC\sys.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "..\..\CNC\INC\cnc.h"

extern CNC_DATA *cncDataPtr;

// ********************************************************************************************
// sysMemMode
// ********************************************************************************************
int sysMemMode(SYS_DATA *sysDataPtr)
{
   switch(sysDataPtr->memState)
   {
   case MEM_IDLE:

	   break;

   case MEM_WAIT_START:
	   break;

   case MEM_RUN:
	   break;

   case MEM_PG_END:
	   break;
   
   default:
	   break;


   }
   
   return 1;
}

// ********************************************************************************************
// sysMain()
// ********************************************************************************************
int sysMain(SYS_DATA *sysDataPtr)
{
int i;
bool ModeReset ;
char *pByte;

/*    ModeReset = linkDataPtr->pannelInput.reset;
    // high priority signal
	if(linkDataPtr->pannelInput.emgStp)
	{
		linkDataPtr->emgStp = true;
		sysDataPtr->sysEmgState = 0 ; // init emg stop
		sysDataPtr->opMode = EMG_STOP;
	}
*/
if(cncDataPtr->linkData.mmiToSys.ncFileValid == true)
{
	sysDataPtr->mmiToSys  = cncDataPtr->linkData.mmiToSys;
	cncDataPtr->linkData.sysToDec.ncFileValid = true;
	strcpy(cncDataPtr->linkData.sysToDec.ncFileName,sysDataPtr->mmiToSys.ncFileName);
	cncDataPtr->linkData.mmiToSys.ncFileValid = false;	
	
}
    
	switch(sysDataPtr->opMode)
	{
	case EMG_STOP:
		break;


		/*case:
			break;
		case:
			break;
		case:
			break;
		case:
			break;
		case:
			break;
		case:
			break;
		default:
			break;*/





	}

	return 1;
}

// ********************************************************************************************
// sysReset()
// ********************************************************************************************
int sysReset(SYS_DATA *sysDataPtr)
{

    return 1;
}


// ********************************************************************************************
// sysRun()
// ********************************************************************************************
int sysRun(SYS_DATA *sysDataPtr)
{

 if(cncDataPtr->linkData.mmiToSys.ncFileValid == true)
 {
   sysDataPtr->mmiToSys  = cncDataPtr->linkData.mmiToSys;
   cncDataPtr->linkData.mmiToSys.ncFileValid = false;
   cncDataPtr->linkData.sysToDec.ncFileValid = sysDataPtr->mmiToSys.ncFileValid;
 }

 switch(sysDataPtr->opMode)
 {
 case MMEM:
     break;

 case MMDI:
	 break;

 case MJOG:
	 break;

 case MWHEEL:
	 break;

 case MTEACH:
	 break;

 case MHOME:
	 break;

 }

 return 1;
}


