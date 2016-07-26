
#ifndef MMIDATAH
#define MMIDATAH
///MMI State
#define MMI_IDLE 0
#define MMI_LIN_PRE 10
#define MMI_LIN_EXE 11
#define MMI_LIN_CLOSE 12

#define MMI_CIR_PRE 20
#define MMI_CIR_EXE 21
#define MMI_CIR_CLOSE 22


///MMI Data Structure
struct MMI_DATA
{
  int mmiState;
  char ncFileName[200];
  bool ncFileValid;

  bool startButton;

  int MDINCLength;
  wchar_t* TextBoxPtr;
};


#endif