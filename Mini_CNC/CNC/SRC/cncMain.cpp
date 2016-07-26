#include "stdafx.h"
#include "..\INC\cnc.h"
#include <stdio.h>
#include <stdlib.h>


extern int    sysInit(SYS_DATA *sysDataPtr);
extern int    decInit(DEC_DATA *decDataPtr);
extern int    intpInit(INTP_DATA *intpDataPtr);
extern int    mmiInit(MMI_DATA *mmiDataPtr);
extern int    linkInit(LINK_DATA *linDataPtr);
extern int    TeachInit(TEACH_DATA *TeachDataPtr);

extern int    mmiCtl(MMI_DATA *mmiDataPtr);
extern int    sysCtl(SYS_DATA *sysDataPtr);
extern int    decCtl(DEC_DATA *decDataPtr);
extern int    intpCtl(INTP_DATA *intpDataPtr);


// ********************************************************************************************
// cncInit()
// ********************************************************************************************
int cncInit(CNC_DATA *cncDataPtr)
{
   sysInit(&cncDataPtr->sysData);
   intpInit(&cncDataPtr->intpData);
   decInit(&cncDataPtr->decData);
   mmiInit(&cncDataPtr->mmiData);   
   /// Initialize all Data Ptr
   linkInit(&cncDataPtr->linkData);
   TeachInit(&cncDataPtr->teachData);

	return 1;
}

// ********************************************************************************************
// cncNrtCtl()   non real time system
// ********************************************************************************************
int cncNrtCtl(CNC_DATA *cncDataPtr)
{
   mmiCtl(&cncDataPtr->mmiData);
   sysCtl(&cncDataPtr->sysData);
   decCtl(&cncDataPtr->decData);

    return 1;
}


// ********************************************************************************************
// cncRtCtl      real time servo
// ********************************************************************************************
int cncRtCtl (CNC_DATA *cncDataPtr)
{

//   pclCtl(&cncDataPtr->mmiData);
  
	intpCtl(&cncDataPtr->intpData);

    return 1;

}