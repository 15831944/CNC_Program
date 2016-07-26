#include <stdio.h>

#ifndef  DECDATAH
#define  DECDATAH

///DEC State
#define DEC_IDLE 0
#define DEC_OPEN_NC_PROGRAM 1
#define DEC_NC_PROGRAM_RUN 2
#define DEC_WRITE_FIFO 3

#define DEC_CLOSE 200

#define DEC_ERROR 100

//G Motive Command
#define G00_RAPID 5
#define G01_LINEAR 6
#define G02_CIR_CW 7
#define G03_CIR_CCW 8
#define G04_DWELL 9

//G Relative Abs command
#define G90_ABS 10
#define G91_REL 11

#define M30_END 9999
#define M01_PAUSE 9998

///DEC Data Structure
struct DEC_DATA
{
  int decState;            //decoder state

  char fileName[100];      //File Name

  FILE *ncFilePtr;         //Openfile pointer

  char FileBuffer[2000];   //content
  char *ascii;             //ascii code pointer

  int Block_num  ;      // for N

  int Block_command  ;  // G code

  double xyz[3] ; // value of x,y,z
  double ijk[3] ;
  double uvw[3] ;
  bool   rel_Motion;         // for G90, G91
  
  double feedrate ;        // feed rate

  bool		markBlock_Command;	// mark for G00, G01, G04
  bool		markXYZ[3];			// mark for X, Y, Z
  bool      markIJK[3];
  bool		markBlock_num;	    // mark for N
  bool		markrelative;		// mark G90, G91
  bool		markfeedrate;		// mark F
  bool      markUVW[3];         // mark for U, V, W
  bool      markM01;            // mark for M function
  bool      markM00;
  bool      markM30;


  bool      markPlane;          // ??

  bool      action;

};


int decIdle( DEC_DATA *decDataPtr );
int decOpenNcFile( DEC_DATA *decDataPtr );
int decNcProgramRun( DEC_DATA *decDataPtr );
int decWrite( DEC_DATA *decDataPtr );
int decReset( DEC_DATA *decDataPtr );

int nFunction( DEC_DATA *decDataPtr );
int gFunction( DEC_DATA *decDataPtr );
int fFunction( DEC_DATA *decDataPtr );
int mFunction( DEC_DATA *decDataPtr );
int coordinateFunction( DEC_DATA *decDataPtr );
int radiusFunction( DEC_DATA *decDataPtr );

int writeToLinkData( DEC_DATA *decDataPtr );
int clearMark( DEC_DATA *decDataPtr );


char* passSpace( char* charPtr );
char* passDigit( char* charPtr );
char* passDigitDot( char* charPtr );
bool  isDigit( char charPtr );
bool  isReal ( char charPtr );

#endif