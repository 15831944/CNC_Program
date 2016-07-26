#include "stdafx.h"
#include "..\INC\dec.h"
#include "..\..\LINK\INC\link.h"
#include <stdio.h>
#include <stdlib.h>

extern int clearMark (DEC_DATA *decDataPtr);

extern LINK_DATA *linkDataPtr;
//  Real Time process has higher priority.

// ********************************************************************************************
// decMain()
// ********************************************************************************************
int decMain(DEC_DATA *decDataPtr)
{
	switch (decDataPtr->decState)
	{
	case DEC_IDLE:
				 decIdle ( decDataPtr );
				 break;

	case DEC_OPEN_NC_PROGRAM:
				 decOpenNcFile ( decDataPtr );
				 linkDataPtr->decToMmi.StateERR = false;
				 linkDataPtr->decToMmi.ERRType = 0;
		     break;

	case DEC_NC_PROGRAM_RUN:
      	 decNcProgramRun ( decDataPtr );
		 	 	 break;

	case DEC_WRITE_FIFO:
	       decWrite(decDataPtr);
         break;

	case DEC_ERROR:
				 decReset(decDataPtr);
				 linkDataPtr->decToMmi.StateERR = true;
				 decDataPtr->decState = DEC_IDLE;
				 break;

	default:
		 //Unknown
		 break;
	}



	return 1;
}

// ********************************************************************************************
// decReset()
// ********************************************************************************************
int decReset(DEC_DATA *decDataPtr)
{
		clearMark ( decDataPtr );
		decDataPtr->decState = DEC_IDLE;

    for( int i = 0 ; i < 3 ; i++)
		{
    decDataPtr->xyz[i] = 0.0;
		decDataPtr->ijk[i] = 0.0;
		decDataPtr->uvw[i] = 0.0;
		}

    return 1;
}
