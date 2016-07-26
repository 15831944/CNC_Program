#include "..\..\LINK\INC\link.h"
#include "..\..\DEC\INC\dec.h"
#include "..\..\INTP\INC\intp.h"
#include "..\..\MMI\INC\mmi.h"
#include "..\..\SYS\INC\sys.h"
#include "..\..\TEACH\INC\TEACH.h"


struct CNC_DATA
{
  SYS_DATA			sysData;
  MMI_DATA			mmiData;
  DEC_DATA			decData;
  INTP_DATA			intpData;
  LINK_DATA		    linkData;
  TEACH_DATA        teachData;
};
