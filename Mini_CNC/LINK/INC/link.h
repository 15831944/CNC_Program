#ifndef LINKH  // to avoid duplicate include
#define LINKH

#define FIFO_BLOCK_MAX 3

#define ERR_MAIN 900
#define ERR_WRITE 901
#define ERR_N 902
#define ERR_G 903
#define ERR_F 904
#define ERR_M 905
#define ERR_C 906

#define MEM 1
#define MDI 2
#define JOG 3
#define WHEEL 4
#define TEACH 5
#define HOME 6






struct MOTION_CTL
{
    bool       feedHold;
	bool       M01;
	bool       singleBlock;
	bool       singleBlockTrigger;
	bool       skip;
	bool       zNeglect;
	bool       dryRun;

	int        feedOverride;

};

struct MMI_TO_DEC
{
	int MDINCLength;
	wchar_t* TextBoxPtr;
};

struct MOTION
{
    double endPoint[3];
	bool   endValid[3];
    bool   relAbs;
	double feed;
	bool   precisionHold;
	int    plane;
	bool   centerValid[3];
	double centerPoint[3];

};

struct DWELL_TIME
{

double value; // G04 F value

};

struct PROGRAM_END
{

};


struct FUNCTION
{
      
	union
	{
		DWELL_TIME  dwellTime;  //G04
		PROGRAM_END programEnd; //M30
	};


};

union MOTION_FUNCTION
{
   MOTION	 motion; //G00 01 02 03 ..
   FUNCTION  function; // G04 M30 ..

 
};

struct DEC_TO_INTP
{
	bool		blockValid;			//標示block是讀或寫狀態

	int			blockNumber;		//行號
	
	int			blockCommand;		//運動模式

	bool		relative;			//相對或絕對運動
	double		feedrate;			//速度
    


	MOTION_FUNCTION   motionFunction;


};

struct DEC_TO_MMI
{
	bool StateERR;
	
	int ERRType; // ERRType : 900- 901- 902- 903- 904- 905-
	
	bool decodeOK;
	


};


struct SYS_TO_DEC
{
    char       ncFileName[100];
	bool       ncFileValid;

};

enum SYS_TO_INTP_CMD {SYS_TO_INTP_NULL = 0 , SYS_TO_INTP_STOP , SYS_TO_INTP_RESET};

struct SYS_TO_INTP
{
	 SYS_TO_INTP_CMD sysToIntpCmd; // stop reset  SYS_TO_INTP_NULL SYS_TO_INTP_STOP, SYS_TO_INTP_RESET


};


struct MMI_TO_SYS
{

	bool       ncFileValid;
    char       ncFileName[100];
	bool       M01switch;
	double        wheelMulti;

	int MDINCLength;
	wchar_t* TextBoxPtr;
};

struct MMI_TO_INTP
{
    bool      feedhold; 
	double    xyzPosition[3];

};

struct FORM_TO_SYS
{
	int        OpMode;
	bool       startButton;

};

struct INTP_TO_MMI
{
	bool		blockValid;
	double		xyzPos[3];
	double      uvwPos[3];
	double      uvwOld[3];
	double      NowSpeed;
	double      xyzTarget[3];
	double      uvwTarget[3];
	bool        programEnd;
};

struct FIFO
{
int counter;
int wrIndex;
int rdIndex;

DEC_TO_INTP block[FIFO_BLOCK_MAX];


};

struct LINK_DATA
{

   SYS_TO_DEC    sysToDec;

   SYS_TO_INTP   sysToIntp;

   DEC_TO_MMI    decToMmi;

   MMI_TO_SYS    mmiToSys;

   MMI_TO_DEC    mmiToDec;

   FORM_TO_SYS   formToSys;

   MOTION_CTL    motionCtl; 

   INTP_TO_MMI   intpToMmi;

   MMI_TO_INTP   mmiToIntp;

   FIFO          BLOCK;

   double timeInterval;

   double standard_feedrate;

};


int linkInit( LINK_DATA *linkDataPtr);






#endif