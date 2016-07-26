#include "stdafx.h"
#include "..\INC\mmi.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "..\..\CNC\INC\cnc.h"

extern CNC_DATA   *cncDataPtr;
extern LINK_DATA  *linkDataPtr;

// ********************************************************************************************
// mmiMain() pack button event and send to sys
// ********************************************************************************************
int mmiMain(MMI_DATA  *mmiDataPtr)
{
	if (mmiDataPtr->ncFileValid == true)
	{
	 if(linkDataPtr->formToSys.OpMode == MEM || linkDataPtr->formToSys.OpMode == HOME)
	 {
	   strcpy(cncDataPtr->linkData.mmiToSys.ncFileName,mmiDataPtr->ncFileName );
	   cncDataPtr->linkData.mmiToSys.ncFileValid = true;
	   mmiDataPtr->ncFileValid = false;
	 }
	 else if (linkDataPtr->formToSys.OpMode == MDI)
	 {
       linkDataPtr->mmiToDec.MDINCLength = mmiDataPtr->MDINCLength;
	   linkDataPtr->mmiToDec.TextBoxPtr = mmiDataPtr->TextBoxPtr;
	   cncDataPtr->linkData.mmiToSys.ncFileValid = true;
	   mmiDataPtr->ncFileValid = false;
	 }
	}
	
    



/*	if (cncDataPtr->linkData.mmiToSysValid ==false)  //block changed
	{
		// cncDataPtr->linkData.mmiToSys = nullMmiToSys;
		// cncDataPtr->linkData.mmiToSys = mmiData.mmiToSysData;
		// cncDataPtr->linkData.mmiToSysValid = true;


	}*/


	return 1;
}

// ********************************************************************************************
// mmiReset()
// ********************************************************************************************
int mmiReset(MMI_DATA *mmiDataPtr)
{
	mmiDataPtr->ncFileValid = false;
    return 1;
}
