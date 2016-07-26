#include "stdafx.h"
#include "..\INC\link.h"


int linkInit( LINK_DATA *linkDataPtr)
{

	linkDataPtr->decToMmi.ERRType =0;
	linkDataPtr->motionCtl.feedHold = false;
	linkDataPtr->decToMmi.StateERR = false;
	linkDataPtr->intpToMmi.programEnd = true;
	linkDataPtr->sysToDec.ncFileValid = false;

	linkDataPtr->standard_feedrate = 30.0;

	for(int i = 0 ; i < 3 ; i++)
	{
    linkDataPtr->BLOCK.block[i].blockValid = false;

	linkDataPtr->intpToMmi.blockValid = false;
	if( i == 0)
	linkDataPtr->intpToMmi.xyzPos[i] = 150.0;

	else
	linkDataPtr->intpToMmi.xyzPos[i] = 0.0;

	linkDataPtr->intpToMmi.uvwPos[i] = 0.0;
	}

	linkDataPtr->motionCtl.singleBlockTrigger = true;
	return 1;
}
