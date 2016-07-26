#include <stdio.h>
#ifndef  INTPDATAH
#define  INTPDATAH
///INTP State
#define INTP_IDLE 0

#define INTP_LIN_PRE 10
#define INTP_LIN_EXE 11
#define INTP_LIN_CLOSE 12

#define INTP_DWELL  104

#define INTP_CIR_PRE 20
#define INTP_CIR_EXE 21
#define INTP_CIR_CLOSE 22


// INTP Ctl State
#define INTP_CTL_RUNNING         500
#define INTP_CTL_RESET_START     510
#define INTP_CTL_RESET_WAIT      511
#define INTP_CTL_RESET_STOP      512
#define INTP_CTL_RESET_ACTION    513
#define INTP_CTL_RESET_WAIT_GO   514   //SYNC WITH OTHER FunctionBlocks
#define INTP_CTL_RESET_WAIT_DEC_FIFO 520

#define SPEED_UP 201
#define SPEED_DOWN 202
#define SPEED_CONST 203

#define MEM 1
#define MDI 2
#define JOG 3
#define WHEEL 4
#define TEACH 5
#define HOME 6

#define MAX_LENGTH 150
#define ARM_LENGTH 75
#define PI 3.14159
struct SLOPE_DATA
{

	int      slopeState;  

	double             Step;

	double             tLength;  
	double             sLength; 
	double             rLength; 

	double             slowdown;

	int                accCount;  
	int                decCount; 
	int                constCount; 

	int                AccGo;  
	int                DecGo; 
	int                ConstGo;

	double             decL;
	double             accL;
	double             constL;

	// INTP
	double             nowSpeed; 
	bool               intpEnd;

	double             getCycle;
	bool               takeoutR;
};


///INTP Data Structure
struct INTP_DATA
{
  int IntpState;

  int IntpCtlState;

  int IntpFeedHold;

  double TimeInterval;

  double Speed;
  double NSpeed;
  // Linear Motion
  bool   relAbs;
  // Position?
  double XYZ_Start[3];
  double XYZ_End[3];
  double UVW_Start[3];
  double UVW_End[3];
  double XYZ_Rel[3];
  double UVW_Rel[3];
  //
  double XYZNew[3];
  double XYZOld[3];
  double UVWNew[3];
  double UVWOld[3];

  double nowPosition[3];
  double nowUVW[3];
  bool Working;

  double centerPoint[3];

  double totalL;
  double restL;
  double nowL;
  double DLength;

  double dwellT;
  int counter;

  double intpFactor[3];

  SLOPE_DATA slopeData;
};




int intp_IDLE(INTP_DATA *intpDataPtr);
int intp_LINPRE(INTP_DATA *intpDataPtr);
int intp_LINEXE(INTP_DATA *intpDataPtr);
int intp_LINCLOSE(INTP_DATA *intpDataPtr);
int intp_CIRPRE(INTP_DATA *intpDataPtr);
int intp_CIREXE(INTP_DATA *intpDataPtr);
int intp_CIRCLOSE(INTP_DATA *intpDataPtr);
int intp_DWELL(INTP_DATA *intpDataPtr);


double Dtrans(double deg);



#endif