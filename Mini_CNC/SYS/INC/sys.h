#include <stdio.h>
#include "..\..\LINK\INC\link.h"
#ifndef SYSDATAH
#define SYSDATAH

///SYS State
#define SYS_IDLE 0
#define SYS_RUN  1
///SYS Data Structure

enum SYS_STATE { sys_INIT = 0 , sys_IDLE , sys_RUN , sys_RESET , sys_HOLD};

enum OP_MODE {MMEM = 1, MMDI , MJOG , MWHEEL , MTEACH , MHOME,EMG_STOP};

enum MEM_STATE {MEM_IDLE = 1, MEM_WAIT_START , MEM_RUN , MEM_PG_END};

struct SYS_DATA
{
  SYS_STATE sysState;
  int resetState;
  OP_MODE opMode;
  MMI_TO_SYS mmiToSys;
  bool       mmiToSysValid;
  MEM_STATE  memState;
};


#endif