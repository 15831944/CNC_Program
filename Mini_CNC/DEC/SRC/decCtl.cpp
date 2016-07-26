#include "stdafx.h"
#include "..\INC\dec.h"
#include "..\..\LINK\INC\link.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern int decMain(DEC_DATA   *decDataPtr);
extern  LINK_DATA   *linkDataPtr;

bool write_fifofull(LINK_DATA   *linkDataPtr);
// ********************************************************************************************
// decInit()
// ********************************************************************************************
int decInit(DEC_DATA *decDataPtr)
{
    decDataPtr -> decState = DEC_IDLE;
	decDataPtr -> ncFilePtr = nullptr;

	clearMark( decDataPtr );
	for( int i = 0 ; i < 3 ; i++)
	{
		decDataPtr->xyz[i] = 0.0;
		decDataPtr->ijk[i] = 0.0;
		decDataPtr->uvw[i] = 0.0;
	}

	linkDataPtr->BLOCK.rdIndex = 0;
	linkDataPtr->BLOCK.wrIndex = 0;
	linkDataPtr->BLOCK.counter = 0;
	return 1;
}

// ********************************************************************************************
// decHandler()
// ********************************************************************************************
int decCtl (DEC_DATA *decDataPtr)
{
	decMain(decDataPtr);
    // process command: Start Stop Cont Reset
	return 1;
}

// ********************************************************************************************
// decClose()
// ********************************************************************************************
int decClose(DEC_DATA *decDataPtr)
{
    return 1;
}

// ********************************************************************************************
// Main function block
// ********************************************************************************************

// IDLE
int decIdle (DEC_DATA *decDataPtr)
{
  if(linkDataPtr->sysToDec.ncFileValid == true)
   {
	if(linkDataPtr->formToSys.OpMode == MEM ||linkDataPtr->formToSys.OpMode == HOME)
	{
	   strcpy(decDataPtr->fileName , linkDataPtr->sysToDec.ncFileName);
     linkDataPtr->sysToDec.ncFileValid = false;
	   decDataPtr->decState = DEC_OPEN_NC_PROGRAM;
	}
	else if (linkDataPtr->formToSys.OpMode == MDI)
	{
		 linkDataPtr->sysToDec.ncFileValid = false;
		 decDataPtr->decState = DEC_OPEN_NC_PROGRAM;
	}
   }

   return 1;
}

// OPEN_NC_FILE
int decOpenNcFile ( DEC_DATA *decDataPtr )
{

 if(linkDataPtr->formToSys.OpMode == MEM || linkDataPtr->formToSys.OpMode == HOME)
   {
	      decDataPtr->ncFilePtr = fopen( decDataPtr->fileName, "r" );

	if( decDataPtr->ncFilePtr == NULL )
	{
        printf( "File Invalid. \n" );
		    decDataPtr->decState = DEC_IDLE;
		    return 0;
	}

	fread( decDataPtr->FileBuffer, sizeof(char) , 2000 , decDataPtr->ncFilePtr );

	decDataPtr->ascii = &decDataPtr->FileBuffer[0];

	fclose ( decDataPtr->ncFilePtr );

	decDataPtr->decState = DEC_NC_PROGRAM_RUN;
  }

 else if (linkDataPtr->formToSys.OpMode == MDI)
	{

		for (int i=0 ; i<linkDataPtr->mmiToDec.MDINCLength ; i++)
		{
			decDataPtr->FileBuffer[i] = linkDataPtr->mmiToDec.TextBoxPtr[i];
		}
		decDataPtr->ascii = &decDataPtr->FileBuffer[0];
		delete [] linkDataPtr->mmiToDec.TextBoxPtr;

		decDataPtr->decState = DEC_NC_PROGRAM_RUN;

	 }

	// to do : if no M30 , read again


    return 1;
}

// NC Program Run

int decNcProgramRun( DEC_DATA *decDataPtr )
{
	bool      EndOfLine = false;
	int		  functionMode = 1;

	while ( EndOfLine == false && functionMode == 1 )
	{
		decDataPtr->ascii = passSpace ( decDataPtr->ascii );

		switch ( *decDataPtr->ascii)
		{
		case 'n':
		case 'N':
			functionMode = nFunction( decDataPtr );
			break;

		case 'g':
		case 'G':
			functionMode = gFunction( decDataPtr );
			break;

		case 'f':
		case 'F':
			functionMode = fFunction( decDataPtr );
            break;

		case 'm':
		case 'M':
			functionMode = mFunction( decDataPtr );
            break;

		case 'x':
		case 'X':
		case 'y':
		case 'Y':
		case 'z':
		case 'Z':
            functionMode = coordinateFunction( decDataPtr );
			break;

		case 'i':
		case 'I':
		case 'j':
		case 'J':
		case 'k':
		case 'K':
            functionMode = radiusFunction ( decDataPtr );
			break;

		case '\n':
		case '\0':
			while ( *decDataPtr->ascii == '\n' )
			{
				decDataPtr->ascii++;
			}

			EndOfLine = true;

			break;

		default:
			printf( "Error: unknown! \n" );
			linkDataPtr->decToMmi.ERRType = ERR_MAIN;
			functionMode = 0;
			break;

		}

	}


	if (functionMode == 1)
	{
        decDataPtr->decState = DEC_WRITE_FIFO;
	}
	else
	{
        decDataPtr->decState = DEC_ERROR;
	}



	return 1;
}

// DEC Write
int decWrite ( DEC_DATA *decDataPtr)
{
    int functionMode = 1;


	if ( decDataPtr->Block_command == 0 )   // unknown status , no writing
	{
       clearMark ( decDataPtr );
	   decDataPtr->decState = DEC_NC_PROGRAM_RUN;
	   return 1;
	}


	if ( linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockValid == false )	//�i�g�J���A
	    {
		// send data
		functionMode = writeToLinkData( decDataPtr );

		if ( functionMode == 1 )
		{
			// set block can be read					//�������]�w�n,�~�iŪ��
			//linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.counter].blockValid = true;

			// check M30
			if ( decDataPtr->Block_command == M30_END )
			{
				// clear marks
				clearMark( decDataPtr );

				decDataPtr->decState = DEC_IDLE;
				decDataPtr->action = false;		//stop dec test-loop
			}
			else
			{
				// clear marks
				clearMark( decDataPtr );

				decDataPtr->decState = DEC_NC_PROGRAM_RUN;

			} // end if command is M30

		}
		else
		{
			decDataPtr->decState = DEC_ERROR;
			linkDataPtr->decToMmi.ERRType = ERR_WRITE;
		} //end if ( functionMode == 1 )

	} //end if ( linkDataPtr->decToIntp.blockValid == false )


    return 1;
}

// clearMark
int clearMark ( DEC_DATA *decDataPtr)
{
    int i;


	decDataPtr->Block_command = NULL;

	decDataPtr->markBlock_Command = false;
	decDataPtr->markBlock_num     = false;
	decDataPtr->markfeedrate      = false;
	decDataPtr->markrelative      = false;
	decDataPtr->markM00           = false;
	decDataPtr->markM01           = false;
	decDataPtr->markM30           = false;
	decDataPtr->markPlane         = false;

	for ( i = 0 ; i < 3 ; i++ )
	{
		decDataPtr->markXYZ[i] = false;
		decDataPtr->markUVW[i] = false;
		decDataPtr->markIJK[i] = false;
	}

    return 1;
}

// nFunction
int nFunction ( DEC_DATA *decDataPtr)
{
	if ( decDataPtr->markBlock_num == true )  // more than one block in one line
	{
		printf( "Error: muliple N!!\n" );
		linkDataPtr->decToMmi.ERRType = ERR_N;
		return 0;
	}

	decDataPtr->ascii++;
	decDataPtr->ascii = passSpace( decDataPtr->ascii );

	if ( isDigit( *decDataPtr->ascii ) == true )
	{
		decDataPtr->Block_num = atoi( decDataPtr->ascii );
		decDataPtr->markBlock_num = true;
		decDataPtr->ascii = passDigit( decDataPtr->ascii );

	}

	else
	{
		printf("ERROR : No value of block number\n");
		linkDataPtr->decToMmi.ERRType = ERR_N;
		return 0;
	}

	return 1;

}

//gFunction
int gFunction ( DEC_DATA *decDataPtr )
{
    int num=0;

	decDataPtr->ascii++;
	decDataPtr->ascii = passSpace( decDataPtr->ascii );

	if ( isDigit( *decDataPtr->ascii ) == true)
	{
		num = atoi( decDataPtr->ascii );

		decDataPtr->ascii = passDigit( decDataPtr->ascii );
	}

	else
	{
		printf("ERROR: No value of Motion Mode!\n");
		linkDataPtr->decToMmi.ERRType = ERR_G;
		return 0;
	}

	//-- function choice
	switch( num )
	{
	case 0: //G00 rapid Motion
		if( decDataPtr->markBlock_Command ==false )
		{
			decDataPtr->Block_command = G00_RAPID;
			decDataPtr->markBlock_Command = true;
		}
		else
		{
			printf("ERROR : multiple command!!\n");
			linkDataPtr->decToMmi.ERRType = ERR_G;
			return 0;
		}

		break;

	case 1:
		if( decDataPtr->markBlock_Command == false )
		{
			decDataPtr->Block_command = G01_LINEAR;
			decDataPtr->markBlock_Command = true;
		}
		else
		{
			printf("ERROR: multiple command!!\n");
			linkDataPtr->decToMmi.ERRType = ERR_G;
			return 0;
		}

		break;

	case 2:
		if( decDataPtr->markBlock_Command == false )
		{
			decDataPtr->Block_command = G02_CIR_CW;
			decDataPtr->markBlock_Command = true;
		}
		else
		{
			printf("ERROR : multiple command!!\n");
			linkDataPtr->decToMmi.ERRType = ERR_G;
			return 0;
		}
		break;

	case 3:
		if( decDataPtr->markBlock_Command == false )
		{
			decDataPtr->Block_command = G03_CIR_CCW;
			decDataPtr->markBlock_Command = true;
		}
		else
		{
			printf("ERROR : multiple command!!\n");
			linkDataPtr->decToMmi.ERRType = ERR_G;
			return 0;
		}
		break;

	case 4:
		if( decDataPtr->markBlock_Command == false)
		{
			decDataPtr->Block_command = G04_DWELL;
			decDataPtr->markBlock_Command = true;
		}
		else
		{
			printf("ERROR: multiple command!!\n");
			linkDataPtr->decToMmi.ERRType = ERR_G;
	        return 0;
		}

		break;

	case 90:
		if (decDataPtr->markrelative == false)
		{
			decDataPtr->markrelative = true;
			decDataPtr->rel_Motion = false;
		}
		else
		{
			printf("ERROR: G90 G91 double command!!\n");
			linkDataPtr->decToMmi.ERRType = ERR_G;
			return 0;
		}

		break;

	case 91:
		if (decDataPtr->markrelative ==false)
		{
			decDataPtr->markrelative = true;
			decDataPtr->rel_Motion = true;
		}

		else
		{
			printf("ERROR: G90 G91 double command!!\n");
			linkDataPtr->decToMmi.ERRType = ERR_G;
			return 0;
		}

		break;

	default:
		printf("Unknown command! \n");
		linkDataPtr->decToMmi.ERRType = ERR_G;
		return 0;

	}


	return 1;
}

//fFunction
int fFunction ( DEC_DATA *decDataPtr)
{
	if (decDataPtr->markfeedrate == true)
	{
        printf("Multiple F command in one line! \n");
		    linkDataPtr->decToMmi.ERRType = ERR_F;
	}


	decDataPtr->ascii++;
	decDataPtr->ascii = passSpace(decDataPtr->ascii);

	if (isReal( *decDataPtr->ascii ) == true )  // check number after F
	{
		  decDataPtr->feedrate = atof(decDataPtr->ascii);
		  decDataPtr->markfeedrate = true;

		  decDataPtr->ascii = passDigitDot (decDataPtr->ascii);

	}

	else
	{
        printf("No number after F!! \n");
		    linkDataPtr->decToMmi.ERRType = ERR_F;
		    return 0;
	}


	return 1;

}

//mFunction
int mFunction(DEC_DATA *decDataPtr)
{
    int num = 0;

	decDataPtr->ascii++;
	decDataPtr->ascii = passSpace( decDataPtr->ascii );


	if ( isDigit( *decDataPtr->ascii ) == true )
	{
		num = atoi (decDataPtr->ascii);

		decDataPtr->ascii = passDigit( decDataPtr->ascii );
	}

	else
	{
    printf("Error M function.\n");
		decDataPtr->decState = DEC_ERROR;
		linkDataPtr->decToMmi.ERRType = ERR_M;
		return 0;
	}

	switch( num )  // M function
	{
	case 30://M30 program end
		if(decDataPtr->markBlock_Command == false)
		{
			decDataPtr->Block_command = M30_END;
			decDataPtr->markBlock_Command = true;
		}

		else
		{
			printf("Error : Multiple command.\n");
			linkDataPtr->decToMmi.ERRType = ERR_M;
			return 0;
		}

		break;
	case 1:
		if(decDataPtr->markBlock_Command == false)
		{
			decDataPtr->Block_command = M01_PAUSE;
			decDataPtr->markBlock_Command = true;
		}

		else
		{
			printf("Error : Multiple command.\n");
			linkDataPtr->decToMmi.ERRType = ERR_M;
			return 0;
		}
		break;

	default:
		printf("Unknown Value After M.\n");
		linkDataPtr->decToMmi.ERRType = ERR_M;
		return 0;

	}

	return 1;
}

//Coordinate Function
int coordinateFunction(DEC_DATA *decDataPtr)
{
	switch( *decDataPtr->ascii )
	{

	case 'x':
	case 'X':
		decDataPtr->ascii++;
		decDataPtr->ascii = passSpace( decDataPtr->ascii );

		if( isReal( *decDataPtr->ascii ) == true )
		{
			if( decDataPtr->markXYZ[0] == false )
			{
				decDataPtr->xyz[0] = atof( decDataPtr->ascii );
				decDataPtr->markXYZ[0] = true;
				decDataPtr->ascii = passDigitDot ( decDataPtr->ascii );
			}

			else
			{
				printf("Multiple X in one line.\n");
				linkDataPtr->decToMmi.ERRType = ERR_C;
				return 0;
			}

		}

		else
		{
			printf("Error: No such number after X.\n");
			linkDataPtr->decToMmi.ERRType = ERR_C;
            return 0;
		}

		break;

	case 'y':
	case 'Y':
		decDataPtr->ascii++;
		decDataPtr->ascii = passSpace( decDataPtr->ascii );

		if( isReal( *decDataPtr->ascii ) == true )
		{
			if( decDataPtr->markXYZ[1] == false )
			{
				decDataPtr->xyz[1] = atof( decDataPtr->ascii );
				decDataPtr->markXYZ[1] = true;
				decDataPtr->ascii = passDigitDot ( decDataPtr->ascii );
			}

			else
			{
				printf("Multiple Y in one line.\n");
				linkDataPtr->decToMmi.ERRType = ERR_C;
				return 0;
			}

		}

		else
		{
			printf("Error: No such number after Y.\n");
			linkDataPtr->decToMmi.ERRType = ERR_C;
            return 0;
		}

		break;

	case 'z':
	case 'Z':
		decDataPtr->ascii++;
		decDataPtr->ascii = passSpace( decDataPtr->ascii );

		if( isReal( *decDataPtr->ascii ) == true )
		{
			if( decDataPtr->markXYZ[2] == false )
			{
				decDataPtr->xyz[2] = atof( decDataPtr->ascii );
				decDataPtr->markXYZ[2] = true;
				decDataPtr->ascii = passDigitDot ( decDataPtr->ascii );
			}

			else
			{
				printf("Multiple Z in one line.\n");
				linkDataPtr->decToMmi.ERRType = ERR_C;
				return 0;
			}

		}

		else
		{
			printf("Error: No such number after Z.\n");
			linkDataPtr->decToMmi.ERRType = ERR_C;
            return 0;
		}

		break;

	default:
         printf("Unable to retrieve Coordinate Data.\n");
		     linkDataPtr->decToMmi.ERRType = ERR_C;
		     return 0;
	}

	return 1;
}

// IJK Function

int radiusFunction(DEC_DATA *decDataPtr)
{
	switch( *decDataPtr->ascii )
	{
	case 'i':
	case 'I':
		decDataPtr->ascii++;
		decDataPtr->ascii = passSpace( decDataPtr->ascii );

		if( isReal( *decDataPtr->ascii ) == true )
		{
			if( decDataPtr->markIJK[0] == false )
			{
				decDataPtr->ijk[0] = atof( decDataPtr->ascii );
				decDataPtr->markIJK[0] = true;
				decDataPtr->ascii = passDigitDot ( decDataPtr->ascii );
			}

			else
			{
				printf("Multiple I in one line.\n");
				linkDataPtr->decToMmi.ERRType = ERR_C;
				return 0;
			}

		}

		else
		{
			printf("Error: No such number after I.\n");
            linkDataPtr->decToMmi.ERRType = ERR_C;
			return 0;
		}


		break;

	case 'j':
	case 'J':
		decDataPtr->ascii++;
		decDataPtr->ascii = passSpace( decDataPtr->ascii );

		if( isReal( *decDataPtr->ascii ) == true )
		{
			if( decDataPtr->markIJK[1] == false )
			{
				decDataPtr->ijk[1] = atof( decDataPtr->ascii );
				decDataPtr->markIJK[1] = true;
				decDataPtr->ascii = passDigitDot ( decDataPtr->ascii );
			}

			else
			{
				printf("Multiple J in one line.\n");
				linkDataPtr->decToMmi.ERRType = ERR_C;
				return 0;
			}

		}

		else
		{
			printf("Error: No such number after J.\n");
			linkDataPtr->decToMmi.ERRType = ERR_C;
            return 0;
		}

		break;

	case 'k':
	case 'K':
		decDataPtr->ascii++;
		decDataPtr->ascii = passSpace( decDataPtr->ascii );

		if( isReal( *decDataPtr->ascii ) == true )
		{
			if( decDataPtr->markIJK[2] == false )
			{
				decDataPtr->ijk[2] = atof( decDataPtr->ascii );
				decDataPtr->markIJK[2] = true;
				decDataPtr->ascii = passDigitDot ( decDataPtr->ascii );
			}

			else
			{
				printf("Multiple K in one line.\n");
				linkDataPtr->decToMmi.ERRType = ERR_C;
				return 0;
			}

		}

		else
		{
			printf("Error: No such number after K.\n");
			linkDataPtr->decToMmi.ERRType = ERR_C;
            return 0;
		}

		break;


	default:
         printf("Unable to retrieve Coordinate Data.\n");
		 linkDataPtr->decToMmi.ERRType = ERR_C;
		 return 0;


	}


   return 1;
}


// passSpace
char* passSpace( char *charPtr)
{
      while ( *charPtr == ' ' || *charPtr == '\t' )
	  {
          charPtr++;
	  }


      return charPtr;

}

// passDigit for simpleFunction
char* passDigit( char *charPtr )
{
      while ( (*charPtr >= '0' && *charPtr <= '9') || *charPtr == '-' )
	  {
          charPtr++;

	  }

	  return charPtr;
}

// passDigit dot for coordinate Function

char* passDigitDot( char *charPtr )
{
      while ( (*charPtr >= '0' && *charPtr <= '9') || *charPtr == '-' || *charPtr == '.')
	  {
          charPtr++;

	  }

	  return charPtr;
}

bool isDigit ( char charPtr)
{
     if ( (charPtr >= '0' && charPtr <= '9')  )
	 {
     return true;
	 }

     return false;
}

bool isReal ( char charPtr)
{
     if ( (charPtr >= '0' && charPtr <= '9') || charPtr == '-' || charPtr == '.' )
	 {
     return true;
	 }

     return false;
}


int writeToLinkData( DEC_DATA *decDataPtr )
{
	int i;
	//style
	// diff fifo diff structure
	// FIFO_HEAD{int fifoType;(important!!) };
 if(write_fifofull(linkDataPtr))
 {
	 linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockNumber = decDataPtr->Block_num;

	switch( decDataPtr->Block_command )
	{
	case G00_RAPID:
	case G01_LINEAR:

		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockCommand = decDataPtr->Block_command;
		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].relative = decDataPtr->rel_Motion;

		if(decDataPtr->markfeedrate)
		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].feedrate = decDataPtr->feedrate;

		else
		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].feedrate = linkDataPtr->standard_feedrate;

		/*// pack G00, G01
		linkDataPtr->decToIntp.blockCommand = decDataPtr->Block_command;
		// pack G90, G91
		linkDataPtr->decToIntp.relative = decDataPtr->rel_Motion;
		// pack F value
		linkDataPtr->decToIntp.feedrate = decDataPtr->feedrate;	*/

		// pack position
		for ( i = 0 ; i < 3 ; i++ )
		{

			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].motionFunction.motion.endValid[i] = decDataPtr->markXYZ[i];
			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].motionFunction.motion.endPoint[i] = decDataPtr->xyz[i];
			/*linkDataPtr->decToIntp.motionFunction.motion.endValid[i] = decDataPtr->markXYZ[i];
			linkDataPtr->decToIntp.motionFunction.motion.endPoint[i] = decDataPtr->xyz[i];*/
		}
		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockValid = true;
		linkDataPtr->BLOCK.counter++;
		linkDataPtr->BLOCK.wrIndex = ++linkDataPtr->BLOCK.wrIndex % FIFO_BLOCK_MAX;

		break;

	case G02_CIR_CW:
	case G03_CIR_CCW:

		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockCommand = decDataPtr->Block_command;
		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].relative = decDataPtr->rel_Motion;

		if(decDataPtr->markfeedrate)
			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].feedrate = decDataPtr->feedrate;

		else
			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].feedrate = linkDataPtr->standard_feedrate;

		for ( i = 0 ; i < 3 ; i++ )
		{

			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].motionFunction.motion.centerValid[i] = decDataPtr->markIJK[i];
			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].motionFunction.motion.centerPoint[i] = decDataPtr->ijk[i];
			/*linkDataPtr->decToIntp.motionFunction.motion.endValid[i] = decDataPtr->markXYZ[i];
			linkDataPtr->decToIntp.motionFunction.motion.endPoint[i] = decDataPtr->xyz[i];*/
		}
		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockValid = true;
		linkDataPtr->BLOCK.counter++;
		linkDataPtr->BLOCK.wrIndex = ++linkDataPtr->BLOCK.wrIndex % FIFO_BLOCK_MAX;

		break;

	case G04_DWELL:
		// G04
		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockCommand = decDataPtr->Block_command;
		/*linkDataPtr->decToIntp.blockCommand = decDataPtr->Block_command;*/
		// pack value
		if ( decDataPtr->markXYZ[0] == true )
		{
			linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].motionFunction.function.dwellTime.value = decDataPtr->xyz[0];
			/*linkDataPtr->decToIntp.motionFunction.function.dwellTime.value = decDataPtr->xyz[0];*/
		}
		else
		{
			printf( "Decoding error: dwell time dead!\n" );
			return 0;
		}
		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockValid = true;
		linkDataPtr->BLOCK.counter++;
		linkDataPtr->BLOCK.wrIndex = ++linkDataPtr->BLOCK.wrIndex % FIFO_BLOCK_MAX;
		break;

	case M30_END:
		//  M30
		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockCommand = decDataPtr->Block_command;

		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockValid = true;
		linkDataPtr->BLOCK.counter++;
		linkDataPtr->BLOCK.wrIndex = ++linkDataPtr->BLOCK.wrIndex % FIFO_BLOCK_MAX;

		break;

	case M01_PAUSE:
		if(linkDataPtr->mmiToSys.M01switch)
		{
    linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockCommand = decDataPtr->Block_command;

		linkDataPtr->BLOCK.block[linkDataPtr->BLOCK.wrIndex].blockValid = true;
		linkDataPtr->BLOCK.counter++;
		linkDataPtr->BLOCK.wrIndex = ++linkDataPtr->BLOCK.wrIndex % FIFO_BLOCK_MAX;
		}

		break;
	default:
		printf( "Packing error: Writing Failure.\n" );
		return 0;
	}

 }


	//

	return 1;
}

bool write_fifofull(LINK_DATA *linkDataPtr)
{
	if (linkDataPtr->BLOCK.counter >= FIFO_BLOCK_MAX)
	{
    return false;
	}
	else
	{
	  return true;
	}
}
