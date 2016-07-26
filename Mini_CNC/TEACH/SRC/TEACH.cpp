#include "stdafx.h"
#include "..\INC\TEACH.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "..\..\CNC\INC\cnc.h"

extern CNC_DATA   *cncDataPtr;
extern LINK_DATA  *linkDataPtr;


void TeachIn(TEACH_DATA * TeachDataPtr);
void TeachRecord(TEACH_DATA *TeachDataPtr);
void TeachEnd(TEACH_DATA * TeachDataPtr);


// ********************************************************************************************
// Teach Main
// ********************************************************************************************
int TeachMain(TEACH_DATA *TeachDataPtr)
{
 switch(TeachDataPtr->TeachState)
 {
 case NOTTEACHING:
	 break;
 
 case TEACHIN:
	 TeachIn(TeachDataPtr);
	 break;
 case RECORD:
	 TeachRecord(TeachDataPtr);
	 break;
 case TEACHEND:
	 TeachEnd(TeachDataPtr);
	 break;
 }

 return 1;


}

// ********************************************************************************************
// TeachInit
// ********************************************************************************************
int TeachInit(TEACH_DATA *TeachDataPtr)
{
	TeachDataPtr->TeachState = NOTTEACHING;
	TeachDataPtr->FileName[0] ='\0';
	return 1;
}

// ********************************************************************************************
// MMIHandler()
// ********************************************************************************************
int TeachCtl (TEACH_DATA *TeachDataPtr)
{
	TeachMain(TeachDataPtr);

	return 1;
}

void TeachIn(TEACH_DATA * TeachDataPtr)
{
	 srand(time(NULL));
     TeachDataPtr->LineNumber = 1;
	 TeachDataPtr->FileName[0] = 'T';
	 TeachDataPtr->FileName[1] = 'e';
	 TeachDataPtr->FileName[2] = 'a';
	 TeachDataPtr->FileName[3] = 'c';
	 TeachDataPtr->FileName[4] = 'h';
	 TeachDataPtr->FileName[5] = '_';
	 for(int i = 0 ; i < 3 ; i++)
	 {
	  int k = rand()%9+1;

	  switch(k)
	  {
	  case 1:
		  TeachDataPtr->FileName[6+i] = '1';
		  break;
	  case 2:
		  TeachDataPtr->FileName[6+i] = '2';
		  break;
	  case 3:
		  TeachDataPtr->FileName[6+i] = '3';
		  break;
	  case 4:
		  TeachDataPtr->FileName[6+i] = '4';
		  break;
	  case 5:
		  TeachDataPtr->FileName[6+i] = '5';
		  break;
	  case 6:
		  TeachDataPtr->FileName[6+i] = '6';
		  break;
	  case 7:
		  TeachDataPtr->FileName[6+i] = '7';
		  break;
	  case 8:
		  TeachDataPtr->FileName[6+i] = '8';
		  break;
	  case 9:
		  TeachDataPtr->FileName[6+i] = '9';
		  break;

	  }
	  TeachDataPtr->FileName[9] = '.';
	  TeachDataPtr->FileName[10] = 't';
	  TeachDataPtr->FileName[11] = 'x';
	  TeachDataPtr->FileName[12] = 't';
	  TeachDataPtr->FileName[13] = '\0';




	 }

	 TeachDataPtr->TeachState = NOTTEACHING;
}

void TeachRecord(TEACH_DATA *TeachDataPtr)
{
	 FILE *FileSave;
	 FileSave = fopen(TeachDataPtr->FileName,"a+");
	 fprintf(FileSave,"N%d ",TeachDataPtr->LineNumber++);
	 fprintf(FileSave,"G01 G90 ");
	 fprintf(FileSave,"X %.3lf ",linkDataPtr->intpToMmi.xyzPos[0]);
	 fprintf(FileSave,"Y %.3lf ",linkDataPtr->intpToMmi.xyzPos[1]);
	 fprintf(FileSave,"Z %.3lf \n",linkDataPtr->intpToMmi.xyzPos[2]);

	 fclose(FileSave);
	 TeachDataPtr->TeachState = NOTTEACHING;
}

void TeachEnd(TEACH_DATA * TeachDataPtr)
{
	 FILE *FileSave;
	 FileSave = fopen(TeachDataPtr->FileName,"a+");
	 fprintf(FileSave,"N%d M30\0",TeachDataPtr->LineNumber++);

	 fclose(FileSave);
	 TeachDataPtr->TeachState = NOTTEACHING;
}
