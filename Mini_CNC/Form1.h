#pragma once

#include "Variables.h"
#include <stdlib.h>
#include <string.h>
#include "CNC\INC\cnc.h"
#include "DRV\INC\MEPCIODEV.H"
#include "DRV\INC\stepMotorDRV.h"
#include "DRV\INC\CP_SWITCH.h"
#include "TEACH\INC\TEACH.h"

CNC_DATA			cncData;

SYS_DATA            *sysDataPtr = &cncData.sysData;
CNC_DATA			*cncDataPtr;
LINK_DATA			*linkDataPtr = &cncData.linkData;
MMI_DATA			*mmiDataPtr = &cncData.mmiData;
DEC_DATA			*decDataPtr = &cncData.decData;
INTP_DATA			*intpDataPtr = &cncData.intpData;
TEACH_DATA          *TeachDataPtr = &cncData.teachData;


extern int decCtl(DEC_DATA *decDataPtr);

extern int cncInit(CNC_DATA *cncDataPtr);
extern int TeachInit(TEACH_DATA *TeachDataPtr);

extern int cncNrtCtl(CNC_DATA *cncDataPtr);
extern int cncRtCtl (CNC_DATA *cncDataPtr);
extern int TeachCtl (TEACH_DATA *TeachDataPtr);


void Init();
void StateOprerator();

extern void trans(double xyz[3],double uvw[3]);

void backTransform(INTP_DATA *intpDataPtr);

typedef struct
{
	double	mSetValue;			// setting value in mm	(or degree)
	double	mActValue;			// actual value in mm	(or degree)
	double	resolution;			// resolution, mm/inc	(or degree/inc)
	int		relInc;				// Relative Increment, 
	                            // relInc = (set - oldSet) / resolution 

	double  remainValue;        // Remainder, in mm (or degree) 

} AXIS_DATA;

AXIS_DATA	axisData[3];
int			xyzButton[3];	//  1 : + mouse down
							// -1 : - mouse down
							//  0 : mouse up


double axisInitValue[3] = {0, 0, 100};

int realKnl(void)
{
	//******************************************
	// position control 
	//******************************************
	cncRtCtl(&cncData);

	
	int i;

	for( i = 0 ; i<3 ; ++i)
	{
	  if(OpMode == MEM || OpMode == MDI || OpMode == HOME )
	  {
		if(i!=2)
		{
		axisData[i].mSetValue = linkDataPtr->intpToMmi.uvwPos[i] *180 / PI;
		axisData[i].mActValue = linkDataPtr->intpToMmi.uvwOld[i] *180 / PI;
		}
		else
		{
		axisData[i].mSetValue = linkDataPtr->intpToMmi.uvwPos[i] ;
		axisData[i].mActValue = linkDataPtr->intpToMmi.uvwOld[i] ;
		}
	  }
	  
		// relDistance = set - oldSet
		double relDistance = axisData[i].mSetValue - axisData[i].mActValue;

		// 	calculate relative increment
		if (relDistance != 0 && !linkDataPtr->motionCtl.feedHold)
		{
			// relInc = (set - oldSet) / resolution
			axisData[i].relInc = (int)(relDistance / axisData[i].resolution);

			// ================== remain value ================== //
			axisData[i].remainValue += ( relDistance - axisData[i].relInc * axisData[i].resolution);
			// remainValue > 0 && resolution <= remainValue
			if( !( axisData[i].resolution > axisData[i].remainValue ) )
			{
				axisData[i].relInc++;
				axisData[i].remainValue -= axisData[i].resolution;
			}
			// remainValue < 0 && resolution <= (-1)*remainValue
			else if(  !( axisData[i].resolution > (-1)*axisData[i].remainValue ) )
			{
				axisData[i].relInc--;
				axisData[i].remainValue += axisData[i].resolution;
			}
			// ================================================== //

		}
		else 
			axisData[i].relInc = 0;

	}//end for

	// output
	for (i = 0 ; i < 3 ; ++i)
	{
		// send to motion card when we use HD_TIMER
#if HD_TIMER
		putAxisValue(i, axisData[i].relInc);
#endif
		axisData[i].mActValue = axisData[i].mSetValue;

	}//end for

	//******************************************
	// intp
	//******************************************
	if( (OpMode == JOG  || OpMode == WHEEL || OpMode == TEACH) && CoordinateState == 0)
	{

	axisData[0].mSetValue += 0.25 * xyzButton[0] * linkDataPtr->mmiToSys.wheelMulti;
	axisData[1].mSetValue += 0.25 * xyzButton[1] * linkDataPtr->mmiToSys.wheelMulti;
	axisData[2].mSetValue += 0.15 * xyzButton[2] * linkDataPtr->mmiToSys.wheelMulti;
    linkDataPtr->intpToMmi.uvwPos[0] += 0.25 * xyzButton[0] * linkDataPtr->mmiToSys.wheelMulti * PI /180;
    linkDataPtr->intpToMmi.uvwPos[1] += 0.25 * xyzButton[1] * linkDataPtr->mmiToSys.wheelMulti * PI /180;
    linkDataPtr->intpToMmi.uvwPos[2] += 0.15 * xyzButton[2] * linkDataPtr->mmiToSys.wheelMulti ;

	if (!CoordinateChange)
	trans(linkDataPtr->intpToMmi.xyzPos,linkDataPtr->intpToMmi.uvwPos);

	for(int q = 0 ; q < 3 ; q++)
	{
		cncDataPtr->intpData.XYZ_Start[q] = linkDataPtr->intpToMmi.xyzPos[q];
		cncDataPtr->intpData.nowPosition[q] = linkDataPtr->intpToMmi.xyzPos[q];
		cncDataPtr->intpData.nowUVW[q] = linkDataPtr->intpToMmi.uvwPos[q];
		cncDataPtr->intpData.UVWOld[q] = linkDataPtr->intpToMmi.uvwPos[q];
	}

	}
	else if ( (OpMode == JOG  || OpMode == WHEEL || OpMode == TEACH ) && CoordinateState == 1)
	{

	linkDataPtr->intpToMmi.xyzPos[0] += 0.01 * xyzButton[0] * linkDataPtr->mmiToSys.wheelMulti;
	linkDataPtr->intpToMmi.xyzPos[1] += 0.01 * xyzButton[1] * linkDataPtr->mmiToSys.wheelMulti;
	linkDataPtr->intpToMmi.xyzPos[2] += 0.15 * xyzButton[2] * linkDataPtr->mmiToSys.wheelMulti;
	cncDataPtr->intpData.nowPosition[0] = linkDataPtr->intpToMmi.xyzPos[0];
	cncDataPtr->intpData.nowPosition[1] = linkDataPtr->intpToMmi.xyzPos[1];
	cncDataPtr->intpData.nowPosition[2] = linkDataPtr->intpToMmi.xyzPos[2];

	if (!CoordinateChange)
	backTransform(&cncDataPtr->intpData);

	linkDataPtr->intpToMmi.uvwPos[0] = cncDataPtr->intpData.nowUVW[0];
	linkDataPtr->intpToMmi.uvwPos[1] = cncDataPtr->intpData.nowUVW[1];
	linkDataPtr->intpToMmi.uvwPos[2] = cncDataPtr->intpData.nowUVW[2];



	for(int q = 0 ; q < 3 ; q++)
	{
		cncDataPtr->intpData.XYZ_Start[q] = linkDataPtr->intpToMmi.xyzPos[q];
		cncDataPtr->intpData.UVWOld[q] = linkDataPtr->intpToMmi.uvwPos[q];
	}

	axisData[0].mSetValue = linkDataPtr->intpToMmi.uvwPos[0];
	axisData[1].mSetValue = linkDataPtr->intpToMmi.uvwPos[1];
	axisData[2].mSetValue = linkDataPtr->intpToMmi.uvwPos[2];


	}


		return 0;
}

//**********************************************
// Real timer function
//**********************************************
#if HD_TIMER
void _stdcall Timer_ISR_Function(LIOINT *pstINTSource)
{
	realKnl();
}
#endif


//**********************************************
//Back Trans
//**********************************************
void backTransform(INTP_DATA *intpDataPtr)
{

	double X;	double Y;	double Z;
	double L1 = 75;
	double L2 = 75;
	double angle1 = 0;//角1-2解1
	double angle2 = 0;//角1-2解2
	double angle3 = 0;//角1+2解1
	double angle4 = 0;//角1+2解2
	double angle5 = 0;//角1 13
	double angle6 = 0;//角2	13
	double angle7 = 0;//角1 14
	double angle8 = 0;//角2 14
	double angle9 = 0;//角1 23
	double angle10 = 0;//角2 23
	double angle11 = 0;//角1 24
	double angle12 = 0;//角2 24
	X = intpDataPtr->nowPosition[0];		Y = intpDataPtr->nowPosition[1];		Z = intpDataPtr->nowPosition[2];

	angle1 = acos( ( X*X + Y*Y - L1*L1 - L2*L2 ) / ( 2*L1*L2 ) );
	if ( angle1 >= PI )
		angle1 -= 2*PI;
	else if( angle1 <= -PI )
		angle1 += 2*PI;

	angle2 = -angle1;//角1-2解2

	if ( angle2 >= PI )
		angle2 -= 2*PI;
	else if( angle2 <= -PI )
		angle2 += 2*PI;

	angle3 = 2 * atan( Y/X );//角1+2解1
	if ( angle3 >= PI )
		angle3 -= 2*PI;
	else if( angle3 <= -PI )
		angle3 += 2*PI;
	angle4 = PI + angle3;//角1+2解2
	if ( angle4 >= PI )
		angle4 -= 2*PI;
	else if( angle4 <= -PI )
		angle4 += 2*PI;
	angle5 = ( angle3 + angle1 )/2;//角1 13
	angle6 = ( angle3 - angle1 )/2;//角2	13

	angle7 = ( angle4 + angle1 )/2;//角1 14
	angle8 = ( angle4 - angle1 )/2;//角2 14

	angle9 = ( angle3 + angle2 )/2;//角1 23
	angle10 = ( angle3 - angle2 )/2;//角2 23

	angle11 = ( angle4 + angle2 )/2;//角1 24
	angle12 = ( angle4 - angle2 )/2;//角2 24


	double temp1;	double temp2;	double temp3;	double temp4;
	double temp5;	double temp6;	double temp7;	double temp8;

	double a;	double b;	double c;	double d;
	temp1 = (intpDataPtr->UVWOld[0] - angle5);
	temp2 = (intpDataPtr->UVWOld[1] - angle6);
	a = temp1 * temp1 + temp2 * temp2;

	temp3 = (intpDataPtr->UVWOld[0] - angle7);
	temp4 = (intpDataPtr->UVWOld[1] - angle8);
	b = temp3 * temp3 + temp4 * temp4;

	temp5 = (intpDataPtr->UVWOld[0] - angle9);
	temp6 = (intpDataPtr->UVWOld[1] - angle10);
	c = temp5 * temp5 + temp6 * temp6;

	temp7 = (intpDataPtr->UVWOld[0] - angle11);
	temp8 = (intpDataPtr->UVWOld[1] - angle12);
	d = temp7 * temp7 + temp8 * temp8;

	if ( a<b && a<c && a<d )
	{
		intpDataPtr->nowUVW[0] = angle5;
		intpDataPtr->nowUVW[1] = angle6;
		intpDataPtr->nowUVW[2] = intpDataPtr->nowPosition[2];
	} 
	else if( b<a && b<c &&b<d )
	{
		intpDataPtr->nowUVW[0] = angle7;
		intpDataPtr->nowUVW[1] = angle8;
		intpDataPtr->nowUVW[2] = intpDataPtr->nowPosition[2];
	}
	else if( c<=a && c<=b && c<=d )
	{
		intpDataPtr->nowUVW[0] = angle9;
		intpDataPtr->nowUVW[1] = angle10;
		intpDataPtr->nowUVW[2] = intpDataPtr->nowPosition[2];
	}
	else if( d<a && d<b && d<c )
	{
		intpDataPtr->nowUVW[0] = angle11;
		intpDataPtr->nowUVW[1] = angle12;
		intpDataPtr->nowUVW[2] = intpDataPtr->nowPosition[2];
	}


}





using namespace System::Runtime::InteropServices;

namespace Mini_CNC {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	
	/// <summary>
	/// Form1 的摘要
	///
	/// 警告: 如果您變更這個類別的名稱，就必須變更與這個類別所依據之所有 .resx 檔案關聯的
	///          Managed 資源編譯器工具的 'Resource File Name' 屬性。
	///          否則，這些設計工具
	///          將無法與這個表單關聯的當地語系化資源
	///          正確互動。
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
		{
			InitializeComponent();
			//
			//TODO: 在此加入建構函式程式碼
			//
		}

	protected:
		/// <summary>
		/// 清除任何使用中的資源。
		/// </summary>
		~Form1()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  button_MEMMode;
	private: System::Windows::Forms::Button^  button_MDIMode;
	private: System::Windows::Forms::Button^  button_JOGMode;
	private: System::Windows::Forms::Button^  button_HOME;
	private: System::Windows::Forms::Button^  button_TEACHMode;
	private: System::Windows::Forms::Button^  button_CoordinateChange;
	private: System::Windows::Forms::Button^  button_EmergencyStop;
	private: System::Windows::Forms::Button^  button_WHEELMode;
	private: System::Windows::Forms::Timer^  timer_NonRealTimer;
	private: System::Windows::Forms::OpenFileDialog^  openFileDialog;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::TrackBar^  trackBar_OverRide;
	private: System::Windows::Forms::Button^  button_OverRideStart;
	private: System::Windows::Forms::Button^  button_OverRideReset;
	private: System::Windows::Forms::Label^  label_FeedOverRide;

	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::Button^  button_SB;
	private: System::Windows::Forms::Button^  button_Wheel_X100;
	private: System::Windows::Forms::Button^  button_FH;
	private: System::Windows::Forms::Button^  button_Wheel_X10;
	private: System::Windows::Forms::Button^  button_M01;
	private: System::Windows::Forms::Button^  button_Wheel_X1;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::Button^  button_ZWCoordinateMinus;
	private: System::Windows::Forms::Button^  button_ZWCoordinatePlus;


	private: System::Windows::Forms::Button^  button_YVCoordinateMinus;
	private: System::Windows::Forms::Button^  button_YVCoordinatePlus;


	private: System::Windows::Forms::Button^  button_XUCoordinateMinus;

	private: System::Windows::Forms::Button^  button_XUCoordinatePlus;
	private: System::Windows::Forms::GroupBox^  groupBox4;
	private: System::Windows::Forms::Button^  button_TeachEnd;
	private: System::Windows::Forms::Button^  button_Record;
	private: System::Windows::Forms::Button^  button_TeachIn;
	private: System::Windows::Forms::GroupBox^  groupBox5;
	private: System::Windows::Forms::Label^  label_WCoordinateValue;

	private: System::Windows::Forms::Label^  label_VCoordinateValue;

	private: System::Windows::Forms::Label^  label_UCoordinateValue;

	private: System::Windows::Forms::Label^  label_W;
	private: System::Windows::Forms::Label^  label_V;
	private: System::Windows::Forms::Label^  label_U;
	private: System::Windows::Forms::GroupBox^  groupBox6;
	private: System::Windows::Forms::Label^  label_Z;

	private: System::Windows::Forms::Label^  label_Y;

	private: System::Windows::Forms::Label^  label_X;
	private: System::Windows::Forms::Label^  label_ZCoordinateValue;

	private: System::Windows::Forms::Label^  label_YCoordinateValue;

	private: System::Windows::Forms::Label^  label_XCoordinateValue;
	private: System::Windows::Forms::RichTextBox^  richTextBox_OpenFileContent;

	private: System::Windows::Forms::Button^  button_OpenFile;
	private: System::Windows::Forms::TextBox^  textBox_filePath;
	private: System::Windows::Forms::Label^  label_M00;
	private: System::Windows::Forms::Label^  label_PGM;
	private: System::Windows::Forms::Label^  label_HY;



	private: System::Windows::Forms::Label^  label_HX;
	private: System::Windows::Forms::Label^  label_PX;


	private: System::Windows::Forms::Label^  label_HZ;

	private: System::Windows::Forms::Label^  label_M30;
	private: System::Windows::Forms::Label^  label_INX;


	private: System::Windows::Forms::Label^  label_ERR;
	private: System::Windows::Forms::Label^  label_INY;
private: System::Windows::Forms::Label^  label_INZ;
private: System::Windows::Forms::Label^  label_NY;




private: System::Windows::Forms::Label^  label_NX;
private: System::Windows::Forms::Label^  label_PZ;
private: System::Windows::Forms::Label^  label_NZ;



private: System::Windows::Forms::Label^  label_PY;
private: System::Windows::Forms::Label^  label_Date;
private: System::Windows::Forms::Label^  label_Time;
private: System::Windows::Forms::PictureBox^  pictureBox1;
private: System::Windows::Forms::Timer^  timer_RealTimer;
private: System::Windows::Forms::Label^  label_Speed;
private: System::Windows::Forms::Button^  button_DEGRAD;














	private: System::ComponentModel::IContainer^  components;
	protected: 

	private:
		/// <summary>
		/// 設計工具所需的變數。
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// 此為設計工具支援所需的方法 - 請勿使用程式碼編輯器修改這個方法的內容。
		///
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(Form1::typeid));
			this->button_MEMMode = (gcnew System::Windows::Forms::Button());
			this->button_MDIMode = (gcnew System::Windows::Forms::Button());
			this->button_JOGMode = (gcnew System::Windows::Forms::Button());
			this->button_HOME = (gcnew System::Windows::Forms::Button());
			this->button_TEACHMode = (gcnew System::Windows::Forms::Button());
			this->button_CoordinateChange = (gcnew System::Windows::Forms::Button());
			this->button_EmergencyStop = (gcnew System::Windows::Forms::Button());
			this->button_WHEELMode = (gcnew System::Windows::Forms::Button());
			this->timer_NonRealTimer = (gcnew System::Windows::Forms::Timer(this->components));
			this->openFileDialog = (gcnew System::Windows::Forms::OpenFileDialog());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label_FeedOverRide = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->trackBar_OverRide = (gcnew System::Windows::Forms::TrackBar());
			this->button_OverRideStart = (gcnew System::Windows::Forms::Button());
			this->button_OverRideReset = (gcnew System::Windows::Forms::Button());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->button_SB = (gcnew System::Windows::Forms::Button());
			this->button_Wheel_X100 = (gcnew System::Windows::Forms::Button());
			this->button_FH = (gcnew System::Windows::Forms::Button());
			this->button_Wheel_X10 = (gcnew System::Windows::Forms::Button());
			this->button_M01 = (gcnew System::Windows::Forms::Button());
			this->button_Wheel_X1 = (gcnew System::Windows::Forms::Button());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->button_ZWCoordinateMinus = (gcnew System::Windows::Forms::Button());
			this->button_ZWCoordinatePlus = (gcnew System::Windows::Forms::Button());
			this->button_YVCoordinateMinus = (gcnew System::Windows::Forms::Button());
			this->button_YVCoordinatePlus = (gcnew System::Windows::Forms::Button());
			this->button_XUCoordinateMinus = (gcnew System::Windows::Forms::Button());
			this->button_XUCoordinatePlus = (gcnew System::Windows::Forms::Button());
			this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
			this->button_TeachEnd = (gcnew System::Windows::Forms::Button());
			this->button_Record = (gcnew System::Windows::Forms::Button());
			this->button_TeachIn = (gcnew System::Windows::Forms::Button());
			this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
			this->label_WCoordinateValue = (gcnew System::Windows::Forms::Label());
			this->label_VCoordinateValue = (gcnew System::Windows::Forms::Label());
			this->label_UCoordinateValue = (gcnew System::Windows::Forms::Label());
			this->label_W = (gcnew System::Windows::Forms::Label());
			this->label_V = (gcnew System::Windows::Forms::Label());
			this->label_U = (gcnew System::Windows::Forms::Label());
			this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
			this->label_ZCoordinateValue = (gcnew System::Windows::Forms::Label());
			this->label_YCoordinateValue = (gcnew System::Windows::Forms::Label());
			this->label_XCoordinateValue = (gcnew System::Windows::Forms::Label());
			this->label_Z = (gcnew System::Windows::Forms::Label());
			this->label_Y = (gcnew System::Windows::Forms::Label());
			this->label_X = (gcnew System::Windows::Forms::Label());
			this->richTextBox_OpenFileContent = (gcnew System::Windows::Forms::RichTextBox());
			this->button_OpenFile = (gcnew System::Windows::Forms::Button());
			this->textBox_filePath = (gcnew System::Windows::Forms::TextBox());
			this->label_M00 = (gcnew System::Windows::Forms::Label());
			this->label_PGM = (gcnew System::Windows::Forms::Label());
			this->label_HY = (gcnew System::Windows::Forms::Label());
			this->label_HX = (gcnew System::Windows::Forms::Label());
			this->label_PX = (gcnew System::Windows::Forms::Label());
			this->label_HZ = (gcnew System::Windows::Forms::Label());
			this->label_M30 = (gcnew System::Windows::Forms::Label());
			this->label_INX = (gcnew System::Windows::Forms::Label());
			this->label_ERR = (gcnew System::Windows::Forms::Label());
			this->label_INY = (gcnew System::Windows::Forms::Label());
			this->label_INZ = (gcnew System::Windows::Forms::Label());
			this->label_NY = (gcnew System::Windows::Forms::Label());
			this->label_NX = (gcnew System::Windows::Forms::Label());
			this->label_PZ = (gcnew System::Windows::Forms::Label());
			this->label_NZ = (gcnew System::Windows::Forms::Label());
			this->label_PY = (gcnew System::Windows::Forms::Label());
			this->label_Date = (gcnew System::Windows::Forms::Label());
			this->label_Time = (gcnew System::Windows::Forms::Label());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->timer_RealTimer = (gcnew System::Windows::Forms::Timer(this->components));
			this->label_Speed = (gcnew System::Windows::Forms::Label());
			this->button_DEGRAD = (gcnew System::Windows::Forms::Button());
			this->groupBox1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar_OverRide))->BeginInit();
			this->groupBox2->SuspendLayout();
			this->groupBox3->SuspendLayout();
			this->groupBox4->SuspendLayout();
			this->groupBox5->SuspendLayout();
			this->groupBox6->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->BeginInit();
			this->SuspendLayout();
			// 
			// button_MEMMode
			// 
			this->button_MEMMode->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->button_MEMMode->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->button_MEMMode->Location = System::Drawing::Point(15, 637);
			this->button_MEMMode->Margin = System::Windows::Forms::Padding(6, 4, 6, 4);
			this->button_MEMMode->Name = L"button_MEMMode";
			this->button_MEMMode->Size = System::Drawing::Size(100, 80);
			this->button_MEMMode->TabIndex = 0;
			this->button_MEMMode->Text = L"MEM";
			this->button_MEMMode->UseVisualStyleBackColor = false;
			this->button_MEMMode->Click += gcnew System::EventHandler(this, &Form1::button_MEMMode_Click);
			// 
			// button_MDIMode
			// 
			this->button_MDIMode->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->button_MDIMode->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->button_MDIMode->Location = System::Drawing::Point(124, 637);
			this->button_MDIMode->Name = L"button_MDIMode";
			this->button_MDIMode->Size = System::Drawing::Size(100, 80);
			this->button_MDIMode->TabIndex = 1;
			this->button_MDIMode->Text = L"MDI";
			this->button_MDIMode->UseVisualStyleBackColor = false;
			this->button_MDIMode->Click += gcnew System::EventHandler(this, &Form1::button_MDIMode_Click);
			// 
			// button_JOGMode
			// 
			this->button_JOGMode->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->button_JOGMode->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->button_JOGMode->Location = System::Drawing::Point(230, 638);
			this->button_JOGMode->Name = L"button_JOGMode";
			this->button_JOGMode->Size = System::Drawing::Size(100, 80);
			this->button_JOGMode->TabIndex = 2;
			this->button_JOGMode->Text = L"JOG";
			this->button_JOGMode->UseVisualStyleBackColor = false;
			this->button_JOGMode->Click += gcnew System::EventHandler(this, &Form1::button_JOGMode_Click);
			// 
			// button_HOME
			// 
			this->button_HOME->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->button_HOME->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->button_HOME->Location = System::Drawing::Point(336, 638);
			this->button_HOME->Name = L"button_HOME";
			this->button_HOME->Size = System::Drawing::Size(100, 80);
			this->button_HOME->TabIndex = 3;
			this->button_HOME->Text = L"HOME";
			this->button_HOME->UseVisualStyleBackColor = false;
			this->button_HOME->Click += gcnew System::EventHandler(this, &Form1::button_HOME_Click);
			// 
			// button_TEACHMode
			// 
			this->button_TEACHMode->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->button_TEACHMode->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->button_TEACHMode->Location = System::Drawing::Point(548, 638);
			this->button_TEACHMode->Name = L"button_TEACHMode";
			this->button_TEACHMode->Size = System::Drawing::Size(100, 80);
			this->button_TEACHMode->TabIndex = 4;
			this->button_TEACHMode->Text = L"TEACH";
			this->button_TEACHMode->UseVisualStyleBackColor = false;
			this->button_TEACHMode->Click += gcnew System::EventHandler(this, &Form1::button_TEACHMode_Click);
			// 
			// button_CoordinateChange
			// 
			this->button_CoordinateChange->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->button_CoordinateChange->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->button_CoordinateChange->Location = System::Drawing::Point(654, 638);
			this->button_CoordinateChange->Name = L"button_CoordinateChange";
			this->button_CoordinateChange->Size = System::Drawing::Size(100, 80);
			this->button_CoordinateChange->TabIndex = 5;
			this->button_CoordinateChange->Text = L"UVW";
			this->button_CoordinateChange->UseVisualStyleBackColor = false;
			this->button_CoordinateChange->Click += gcnew System::EventHandler(this, &Form1::button_CoordinateChange_Click);
			// 
			// button_EmergencyStop
			// 
			this->button_EmergencyStop->BackColor = System::Drawing::Color::SandyBrown;
			this->button_EmergencyStop->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->button_EmergencyStop->Location = System::Drawing::Point(780, 638);
			this->button_EmergencyStop->Name = L"button_EmergencyStop";
			this->button_EmergencyStop->Size = System::Drawing::Size(200, 80);
			this->button_EmergencyStop->TabIndex = 6;
			this->button_EmergencyStop->Text = L"EmergencyStop";
			this->button_EmergencyStop->UseVisualStyleBackColor = false;
			this->button_EmergencyStop->Click += gcnew System::EventHandler(this, &Form1::button_EmergencyStop_Click);
			// 
			// button_WHEELMode
			// 
			this->button_WHEELMode->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->button_WHEELMode->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->button_WHEELMode->Location = System::Drawing::Point(442, 638);
			this->button_WHEELMode->Name = L"button_WHEELMode";
			this->button_WHEELMode->Size = System::Drawing::Size(100, 80);
			this->button_WHEELMode->TabIndex = 7;
			this->button_WHEELMode->Text = L"WHEEL";
			this->button_WHEELMode->UseVisualStyleBackColor = false;
			this->button_WHEELMode->Click += gcnew System::EventHandler(this, &Form1::button_WHEELMode_Click);
			// 
			// timer_NonRealTimer
			// 
			this->timer_NonRealTimer->Tick += gcnew System::EventHandler(this, &Form1::timer_NonRealTimer_Tick);
			// 
			// openFileDialog
			// 
			this->openFileDialog->FileName = L"openFileDialog1";
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->label_FeedOverRide);
			this->groupBox1->Controls->Add(this->label1);
			this->groupBox1->Controls->Add(this->trackBar_OverRide);
			this->groupBox1->Controls->Add(this->button_OverRideStart);
			this->groupBox1->Controls->Add(this->button_OverRideReset);
			this->groupBox1->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->groupBox1->Location = System::Drawing::Point(730, 418);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(250, 189);
			this->groupBox1->TabIndex = 8;
			this->groupBox1->TabStop = false;
			// 
			// label_FeedOverRide
			// 
			this->label_FeedOverRide->AutoSize = true;
			this->label_FeedOverRide->Location = System::Drawing::Point(153, 27);
			this->label_FeedOverRide->Name = L"label_FeedOverRide";
			this->label_FeedOverRide->Size = System::Drawing::Size(39, 18);
			this->label_FeedOverRide->TabIndex = 4;
			this->label_FeedOverRide->Text = L"0%";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(17, 27);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(130, 18);
			this->label1->TabIndex = 3;
			this->label1->Text = L"FreeOverRide";
			// 
			// trackBar_OverRide
			// 
			this->trackBar_OverRide->Location = System::Drawing::Point(12, 71);
			this->trackBar_OverRide->Maximum = 200;
			this->trackBar_OverRide->Name = L"trackBar_OverRide";
			this->trackBar_OverRide->Size = System::Drawing::Size(238, 45);
			this->trackBar_OverRide->TabIndex = 2;
			this->trackBar_OverRide->TickFrequency = 10;
			this->trackBar_OverRide->Value = 100;
			this->trackBar_OverRide->Scroll += gcnew System::EventHandler(this, &Form1::trackBar_OverRide_Scroll);
			// 
			// button_OverRideStart
			// 
			this->button_OverRideStart->Location = System::Drawing::Point(146, 128);
			this->button_OverRideStart->Name = L"button_OverRideStart";
			this->button_OverRideStart->Size = System::Drawing::Size(80, 30);
			this->button_OverRideStart->TabIndex = 1;
			this->button_OverRideStart->Text = L"Start";
			this->button_OverRideStart->UseVisualStyleBackColor = true;
			this->button_OverRideStart->Click += gcnew System::EventHandler(this, &Form1::button_OverRideStart_Click);
			// 
			// button_OverRideReset
			// 
			this->button_OverRideReset->Location = System::Drawing::Point(29, 128);
			this->button_OverRideReset->Name = L"button_OverRideReset";
			this->button_OverRideReset->Size = System::Drawing::Size(80, 30);
			this->button_OverRideReset->TabIndex = 0;
			this->button_OverRideReset->Text = L"Reset";
			this->button_OverRideReset->UseVisualStyleBackColor = true;
			this->button_OverRideReset->Click += gcnew System::EventHandler(this, &Form1::button_OverRideReset_Click);
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->button_SB);
			this->groupBox2->Controls->Add(this->button_Wheel_X100);
			this->groupBox2->Controls->Add(this->button_FH);
			this->groupBox2->Controls->Add(this->button_Wheel_X10);
			this->groupBox2->Controls->Add(this->button_M01);
			this->groupBox2->Controls->Add(this->button_Wheel_X1);
			this->groupBox2->Location = System::Drawing::Point(525, 418);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(170, 190);
			this->groupBox2->TabIndex = 9;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Functions";
			// 
			// button_SB
			// 
			this->button_SB->Location = System::Drawing::Point(89, 140);
			this->button_SB->Name = L"button_SB";
			this->button_SB->Size = System::Drawing::Size(70, 35);
			this->button_SB->TabIndex = 5;
			this->button_SB->Text = L"SB";
			this->button_SB->UseVisualStyleBackColor = true;
			this->button_SB->Click += gcnew System::EventHandler(this, &Form1::button_SB_Click);
			// 
			// button_Wheel_X100
			// 
			this->button_Wheel_X100->Location = System::Drawing::Point(6, 140);
			this->button_Wheel_X100->Name = L"button_Wheel_X100";
			this->button_Wheel_X100->Size = System::Drawing::Size(70, 35);
			this->button_Wheel_X100->TabIndex = 4;
			this->button_Wheel_X100->Text = L"X100";
			this->button_Wheel_X100->UseVisualStyleBackColor = true;
			this->button_Wheel_X100->Click += gcnew System::EventHandler(this, &Form1::button_Wheel_X100_Click);
			// 
			// button_FH
			// 
			this->button_FH->Location = System::Drawing::Point(89, 81);
			this->button_FH->Name = L"button_FH";
			this->button_FH->Size = System::Drawing::Size(70, 35);
			this->button_FH->TabIndex = 3;
			this->button_FH->Text = L"FH";
			this->button_FH->UseVisualStyleBackColor = true;
			this->button_FH->Click += gcnew System::EventHandler(this, &Form1::button_FH_Click);
			// 
			// button_Wheel_X10
			// 
			this->button_Wheel_X10->Location = System::Drawing::Point(6, 81);
			this->button_Wheel_X10->Name = L"button_Wheel_X10";
			this->button_Wheel_X10->Size = System::Drawing::Size(70, 35);
			this->button_Wheel_X10->TabIndex = 2;
			this->button_Wheel_X10->Text = L"X10";
			this->button_Wheel_X10->UseVisualStyleBackColor = true;
			this->button_Wheel_X10->Click += gcnew System::EventHandler(this, &Form1::button_Wheel_X10_Click);
			// 
			// button_M01
			// 
			this->button_M01->Location = System::Drawing::Point(89, 21);
			this->button_M01->Name = L"button_M01";
			this->button_M01->Size = System::Drawing::Size(70, 35);
			this->button_M01->TabIndex = 1;
			this->button_M01->Text = L"M01";
			this->button_M01->UseVisualStyleBackColor = true;
			this->button_M01->Click += gcnew System::EventHandler(this, &Form1::button_M01_Click);
			// 
			// button_Wheel_X1
			// 
			this->button_Wheel_X1->Location = System::Drawing::Point(6, 21);
			this->button_Wheel_X1->Name = L"button_Wheel_X1";
			this->button_Wheel_X1->Size = System::Drawing::Size(70, 35);
			this->button_Wheel_X1->TabIndex = 0;
			this->button_Wheel_X1->Text = L"X1";
			this->button_Wheel_X1->UseVisualStyleBackColor = true;
			this->button_Wheel_X1->Click += gcnew System::EventHandler(this, &Form1::button_Wheel_X1_Click);
			// 
			// groupBox3
			// 
			this->groupBox3->Controls->Add(this->button_ZWCoordinateMinus);
			this->groupBox3->Controls->Add(this->button_ZWCoordinatePlus);
			this->groupBox3->Controls->Add(this->button_YVCoordinateMinus);
			this->groupBox3->Controls->Add(this->button_YVCoordinatePlus);
			this->groupBox3->Controls->Add(this->button_XUCoordinateMinus);
			this->groupBox3->Controls->Add(this->button_XUCoordinatePlus);
			this->groupBox3->Location = System::Drawing::Point(349, 418);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(170, 190);
			this->groupBox3->TabIndex = 10;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"JOG";
			// 
			// button_ZWCoordinateMinus
			// 
			this->button_ZWCoordinateMinus->Location = System::Drawing::Point(93, 140);
			this->button_ZWCoordinateMinus->Name = L"button_ZWCoordinateMinus";
			this->button_ZWCoordinateMinus->Size = System::Drawing::Size(70, 35);
			this->button_ZWCoordinateMinus->TabIndex = 5;
			this->button_ZWCoordinateMinus->Text = L"Z-";
			this->button_ZWCoordinateMinus->UseVisualStyleBackColor = true;
			this->button_ZWCoordinateMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_ZWCoordinateMinus_MouseDown);
			this->button_ZWCoordinateMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_ZWCoordinateMinus_MouseUp);
			// 
			// button_ZWCoordinatePlus
			// 
			this->button_ZWCoordinatePlus->Location = System::Drawing::Point(17, 140);
			this->button_ZWCoordinatePlus->Name = L"button_ZWCoordinatePlus";
			this->button_ZWCoordinatePlus->Size = System::Drawing::Size(70, 35);
			this->button_ZWCoordinatePlus->TabIndex = 4;
			this->button_ZWCoordinatePlus->Text = L"Z+";
			this->button_ZWCoordinatePlus->UseVisualStyleBackColor = true;
			this->button_ZWCoordinatePlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_ZWCoordinatePlus_MouseDown);
			this->button_ZWCoordinatePlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_ZWCoordinatePlus_MouseUp);
			// 
			// button_YVCoordinateMinus
			// 
			this->button_YVCoordinateMinus->Location = System::Drawing::Point(94, 81);
			this->button_YVCoordinateMinus->Name = L"button_YVCoordinateMinus";
			this->button_YVCoordinateMinus->Size = System::Drawing::Size(70, 35);
			this->button_YVCoordinateMinus->TabIndex = 3;
			this->button_YVCoordinateMinus->Text = L"Y-";
			this->button_YVCoordinateMinus->UseVisualStyleBackColor = true;
			this->button_YVCoordinateMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_YVCoordinateMinus_MouseDown);
			this->button_YVCoordinateMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_YVCoordinateMinus_MouseUp);
			// 
			// button_YVCoordinatePlus
			// 
			this->button_YVCoordinatePlus->Location = System::Drawing::Point(17, 81);
			this->button_YVCoordinatePlus->Name = L"button_YVCoordinatePlus";
			this->button_YVCoordinatePlus->Size = System::Drawing::Size(70, 35);
			this->button_YVCoordinatePlus->TabIndex = 2;
			this->button_YVCoordinatePlus->Text = L"Y+";
			this->button_YVCoordinatePlus->UseVisualStyleBackColor = true;
			this->button_YVCoordinatePlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_YVCoordinatePlus_MouseDown);
			this->button_YVCoordinatePlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_YVCoordinatePlus_MouseUp);
			// 
			// button_XUCoordinateMinus
			// 
			this->button_XUCoordinateMinus->Location = System::Drawing::Point(93, 21);
			this->button_XUCoordinateMinus->Name = L"button_XUCoordinateMinus";
			this->button_XUCoordinateMinus->Size = System::Drawing::Size(70, 35);
			this->button_XUCoordinateMinus->TabIndex = 1;
			this->button_XUCoordinateMinus->Text = L"X-";
			this->button_XUCoordinateMinus->UseVisualStyleBackColor = true;
			this->button_XUCoordinateMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_XUCoordinateMinus_MouseDown);
			this->button_XUCoordinateMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_XUCoordinateMinus_MouseUp);
			// 
			// button_XUCoordinatePlus
			// 
			this->button_XUCoordinatePlus->Location = System::Drawing::Point(17, 21);
			this->button_XUCoordinatePlus->Name = L"button_XUCoordinatePlus";
			this->button_XUCoordinatePlus->Size = System::Drawing::Size(70, 35);
			this->button_XUCoordinatePlus->TabIndex = 0;
			this->button_XUCoordinatePlus->Text = L"X+";
			this->button_XUCoordinatePlus->UseVisualStyleBackColor = true;
			this->button_XUCoordinatePlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_XUCoordinatePlus_MouseDown);
			this->button_XUCoordinatePlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::button_XUCoordinatePlus_MouseUp);
			// 
			// groupBox4
			// 
			this->groupBox4->Controls->Add(this->button_TeachEnd);
			this->groupBox4->Controls->Add(this->button_Record);
			this->groupBox4->Controls->Add(this->button_TeachIn);
			this->groupBox4->Location = System::Drawing::Point(221, 418);
			this->groupBox4->Name = L"groupBox4";
			this->groupBox4->Size = System::Drawing::Size(122, 189);
			this->groupBox4->TabIndex = 11;
			this->groupBox4->TabStop = false;
			this->groupBox4->Text = L"TEACH";
			// 
			// button_TeachEnd
			// 
			this->button_TeachEnd->Font = (gcnew System::Drawing::Font(L"Verdana", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button_TeachEnd->Location = System::Drawing::Point(16, 135);
			this->button_TeachEnd->Name = L"button_TeachEnd";
			this->button_TeachEnd->Size = System::Drawing::Size(100, 40);
			this->button_TeachEnd->TabIndex = 2;
			this->button_TeachEnd->Text = L"TEACH END";
			this->button_TeachEnd->UseVisualStyleBackColor = true;
			this->button_TeachEnd->Click += gcnew System::EventHandler(this, &Form1::button_TeachEnd_Click);
			// 
			// button_Record
			// 
			this->button_Record->Font = (gcnew System::Drawing::Font(L"Verdana", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button_Record->Location = System::Drawing::Point(16, 78);
			this->button_Record->Name = L"button_Record";
			this->button_Record->Size = System::Drawing::Size(100, 40);
			this->button_Record->TabIndex = 1;
			this->button_Record->Text = L"Record";
			this->button_Record->UseVisualStyleBackColor = true;
			this->button_Record->Click += gcnew System::EventHandler(this, &Form1::button_Record_Click);
			// 
			// button_TeachIn
			// 
			this->button_TeachIn->Font = (gcnew System::Drawing::Font(L"Verdana", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button_TeachIn->Location = System::Drawing::Point(16, 25);
			this->button_TeachIn->Name = L"button_TeachIn";
			this->button_TeachIn->Size = System::Drawing::Size(100, 40);
			this->button_TeachIn->TabIndex = 0;
			this->button_TeachIn->Text = L"TEACH IN";
			this->button_TeachIn->UseVisualStyleBackColor = true;
			this->button_TeachIn->Click += gcnew System::EventHandler(this, &Form1::button_TeachIn_Click);
			// 
			// groupBox5
			// 
			this->groupBox5->Controls->Add(this->button_DEGRAD);
			this->groupBox5->Controls->Add(this->label_WCoordinateValue);
			this->groupBox5->Controls->Add(this->label_VCoordinateValue);
			this->groupBox5->Controls->Add(this->label_UCoordinateValue);
			this->groupBox5->Controls->Add(this->label_W);
			this->groupBox5->Controls->Add(this->label_V);
			this->groupBox5->Controls->Add(this->label_U);
			this->groupBox5->ImeMode = System::Windows::Forms::ImeMode::NoControl;
			this->groupBox5->Location = System::Drawing::Point(7, 418);
			this->groupBox5->Name = L"groupBox5";
			this->groupBox5->Size = System::Drawing::Size(208, 189);
			this->groupBox5->TabIndex = 12;
			this->groupBox5->TabStop = false;
			this->groupBox5->Text = L"MC";
			// 
			// label_WCoordinateValue
			// 
			this->label_WCoordinateValue->BackColor = System::Drawing::Color::PowderBlue;
			this->label_WCoordinateValue->Location = System::Drawing::Point(62, 133);
			this->label_WCoordinateValue->Name = L"label_WCoordinateValue";
			this->label_WCoordinateValue->Size = System::Drawing::Size(130, 40);
			this->label_WCoordinateValue->TabIndex = 5;
			this->label_WCoordinateValue->Text = L"0.0";
			this->label_WCoordinateValue->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_VCoordinateValue
			// 
			this->label_VCoordinateValue->BackColor = System::Drawing::Color::PowderBlue;
			this->label_VCoordinateValue->Location = System::Drawing::Point(62, 83);
			this->label_VCoordinateValue->Name = L"label_VCoordinateValue";
			this->label_VCoordinateValue->Size = System::Drawing::Size(130, 40);
			this->label_VCoordinateValue->TabIndex = 4;
			this->label_VCoordinateValue->Text = L"0.0";
			this->label_VCoordinateValue->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_UCoordinateValue
			// 
			this->label_UCoordinateValue->BackColor = System::Drawing::Color::PowderBlue;
			this->label_UCoordinateValue->Location = System::Drawing::Point(62, 35);
			this->label_UCoordinateValue->Name = L"label_UCoordinateValue";
			this->label_UCoordinateValue->Size = System::Drawing::Size(130, 40);
			this->label_UCoordinateValue->TabIndex = 3;
			this->label_UCoordinateValue->Text = L"0.0";
			this->label_UCoordinateValue->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_W
			// 
			this->label_W->BackColor = System::Drawing::SystemColors::Highlight;
			this->label_W->Location = System::Drawing::Point(16, 133);
			this->label_W->Name = L"label_W";
			this->label_W->Size = System::Drawing::Size(40, 40);
			this->label_W->TabIndex = 2;
			this->label_W->Text = L"W";
			this->label_W->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_V
			// 
			this->label_V->BackColor = System::Drawing::SystemColors::Highlight;
			this->label_V->Location = System::Drawing::Point(16, 83);
			this->label_V->Name = L"label_V";
			this->label_V->Size = System::Drawing::Size(40, 40);
			this->label_V->TabIndex = 1;
			this->label_V->Text = L"V";
			this->label_V->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_U
			// 
			this->label_U->BackColor = System::Drawing::SystemColors::Highlight;
			this->label_U->Location = System::Drawing::Point(16, 35);
			this->label_U->Name = L"label_U";
			this->label_U->Size = System::Drawing::Size(40, 40);
			this->label_U->TabIndex = 0;
			this->label_U->Text = L"U";
			this->label_U->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// groupBox6
			// 
			this->groupBox6->Controls->Add(this->label_ZCoordinateValue);
			this->groupBox6->Controls->Add(this->label_YCoordinateValue);
			this->groupBox6->Controls->Add(this->label_XCoordinateValue);
			this->groupBox6->Controls->Add(this->label_Z);
			this->groupBox6->Controls->Add(this->label_Y);
			this->groupBox6->Controls->Add(this->label_X);
			this->groupBox6->Location = System::Drawing::Point(7, 125);
			this->groupBox6->Name = L"groupBox6";
			this->groupBox6->Size = System::Drawing::Size(336, 285);
			this->groupBox6->TabIndex = 13;
			this->groupBox6->TabStop = false;
			// 
			// label_ZCoordinateValue
			// 
			this->label_ZCoordinateValue->BackColor = System::Drawing::Color::PowderBlue;
			this->label_ZCoordinateValue->FlatStyle = System::Windows::Forms::FlatStyle::Flat;
			this->label_ZCoordinateValue->Font = (gcnew System::Drawing::Font(L"Verdana", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label_ZCoordinateValue->Location = System::Drawing::Point(114, 198);
			this->label_ZCoordinateValue->Name = L"label_ZCoordinateValue";
			this->label_ZCoordinateValue->Size = System::Drawing::Size(200, 65);
			this->label_ZCoordinateValue->TabIndex = 5;
			this->label_ZCoordinateValue->Text = L"0.0";
			this->label_ZCoordinateValue->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_YCoordinateValue
			// 
			this->label_YCoordinateValue->BackColor = System::Drawing::Color::PowderBlue;
			this->label_YCoordinateValue->FlatStyle = System::Windows::Forms::FlatStyle::Flat;
			this->label_YCoordinateValue->Font = (gcnew System::Drawing::Font(L"Verdana", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label_YCoordinateValue->Location = System::Drawing::Point(114, 114);
			this->label_YCoordinateValue->Name = L"label_YCoordinateValue";
			this->label_YCoordinateValue->Size = System::Drawing::Size(200, 65);
			this->label_YCoordinateValue->TabIndex = 4;
			this->label_YCoordinateValue->Text = L"0.0";
			this->label_YCoordinateValue->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_XCoordinateValue
			// 
			this->label_XCoordinateValue->BackColor = System::Drawing::Color::PowderBlue;
			this->label_XCoordinateValue->FlatStyle = System::Windows::Forms::FlatStyle::Flat;
			this->label_XCoordinateValue->Font = (gcnew System::Drawing::Font(L"Verdana", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label_XCoordinateValue->Location = System::Drawing::Point(114, 32);
			this->label_XCoordinateValue->Name = L"label_XCoordinateValue";
			this->label_XCoordinateValue->Size = System::Drawing::Size(200, 65);
			this->label_XCoordinateValue->TabIndex = 3;
			this->label_XCoordinateValue->Text = L"0.0";
			this->label_XCoordinateValue->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_Z
			// 
			this->label_Z->BackColor = System::Drawing::SystemColors::Highlight;
			this->label_Z->Font = (gcnew System::Drawing::Font(L"Verdana", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label_Z->Location = System::Drawing::Point(22, 198);
			this->label_Z->Name = L"label_Z";
			this->label_Z->Size = System::Drawing::Size(65, 65);
			this->label_Z->TabIndex = 2;
			this->label_Z->Text = L"Z";
			this->label_Z->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_Y
			// 
			this->label_Y->BackColor = System::Drawing::SystemColors::Highlight;
			this->label_Y->Font = (gcnew System::Drawing::Font(L"Verdana", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label_Y->Location = System::Drawing::Point(22, 114);
			this->label_Y->Name = L"label_Y";
			this->label_Y->Size = System::Drawing::Size(65, 65);
			this->label_Y->TabIndex = 1;
			this->label_Y->Text = L"Y";
			this->label_Y->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_X
			// 
			this->label_X->BackColor = System::Drawing::SystemColors::Highlight;
			this->label_X->Font = (gcnew System::Drawing::Font(L"Verdana", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label_X->Location = System::Drawing::Point(22, 32);
			this->label_X->Name = L"label_X";
			this->label_X->Size = System::Drawing::Size(65, 65);
			this->label_X->TabIndex = 0;
			this->label_X->Text = L"X";
			this->label_X->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// richTextBox_OpenFileContent
			// 
			this->richTextBox_OpenFileContent->BackColor = System::Drawing::SystemColors::ScrollBar;
			this->richTextBox_OpenFileContent->Location = System::Drawing::Point(349, 133);
			this->richTextBox_OpenFileContent->Name = L"richTextBox_OpenFileContent";
			this->richTextBox_OpenFileContent->ReadOnly = true;
			this->richTextBox_OpenFileContent->Size = System::Drawing::Size(631, 235);
			this->richTextBox_OpenFileContent->TabIndex = 14;
			this->richTextBox_OpenFileContent->Text = L"";
			// 
			// button_OpenFile
			// 
			this->button_OpenFile->Location = System::Drawing::Point(349, 377);
			this->button_OpenFile->Name = L"button_OpenFile";
			this->button_OpenFile->Size = System::Drawing::Size(110, 35);
			this->button_OpenFile->TabIndex = 15;
			this->button_OpenFile->Text = L"Open";
			this->button_OpenFile->UseVisualStyleBackColor = true;
			this->button_OpenFile->Click += gcnew System::EventHandler(this, &Form1::button_OpenFile_Click);
			// 
			// textBox_filePath
			// 
			this->textBox_filePath->BackColor = System::Drawing::SystemColors::ScrollBar;
			this->textBox_filePath->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->textBox_filePath->Location = System::Drawing::Point(465, 383);
			this->textBox_filePath->Name = L"textBox_filePath";
			this->textBox_filePath->Size = System::Drawing::Size(515, 27);
			this->textBox_filePath->TabIndex = 16;
			// 
			// label_M00
			// 
			this->label_M00->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_M00->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_M00->Location = System::Drawing::Point(141, 13);
			this->label_M00->Name = L"label_M00";
			this->label_M00->Size = System::Drawing::Size(85, 45);
			this->label_M00->TabIndex = 17;
			this->label_M00->Text = L"M00";
			this->label_M00->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_PGM
			// 
			this->label_PGM->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_PGM->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_PGM->Location = System::Drawing::Point(226, 13);
			this->label_PGM->Name = L"label_PGM";
			this->label_PGM->Size = System::Drawing::Size(85, 45);
			this->label_PGM->TabIndex = 18;
			this->label_PGM->Text = L"PGM";
			this->label_PGM->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_HY
			// 
			this->label_HY->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_HY->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_HY->Location = System::Drawing::Point(396, 13);
			this->label_HY->Name = L"label_HY";
			this->label_HY->Size = System::Drawing::Size(85, 45);
			this->label_HY->TabIndex = 19;
			this->label_HY->Text = L"HY";
			this->label_HY->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_HX
			// 
			this->label_HX->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_HX->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_HX->Location = System::Drawing::Point(311, 13);
			this->label_HX->Name = L"label_HX";
			this->label_HX->Size = System::Drawing::Size(85, 45);
			this->label_HX->TabIndex = 20;
			this->label_HX->Text = L"HX";
			this->label_HX->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_PX
			// 
			this->label_PX->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_PX->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_PX->Location = System::Drawing::Point(566, 13);
			this->label_PX->Name = L"label_PX";
			this->label_PX->Size = System::Drawing::Size(85, 45);
			this->label_PX->TabIndex = 21;
			this->label_PX->Text = L"PX";
			this->label_PX->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_HZ
			// 
			this->label_HZ->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_HZ->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_HZ->Location = System::Drawing::Point(481, 13);
			this->label_HZ->Name = L"label_HZ";
			this->label_HZ->Size = System::Drawing::Size(85, 45);
			this->label_HZ->TabIndex = 22;
			this->label_HZ->Text = L"HZ";
			this->label_HZ->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_M30
			// 
			this->label_M30->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_M30->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_M30->Location = System::Drawing::Point(141, 58);
			this->label_M30->Name = L"label_M30";
			this->label_M30->Size = System::Drawing::Size(85, 45);
			this->label_M30->TabIndex = 23;
			this->label_M30->Text = L"M30";
			this->label_M30->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_INX
			// 
			this->label_INX->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_INX->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_INX->Location = System::Drawing::Point(311, 58);
			this->label_INX->Name = L"label_INX";
			this->label_INX->Size = System::Drawing::Size(85, 45);
			this->label_INX->TabIndex = 24;
			this->label_INX->Text = L"INX";
			this->label_INX->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_ERR
			// 
			this->label_ERR->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_ERR->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_ERR->Location = System::Drawing::Point(226, 58);
			this->label_ERR->Name = L"label_ERR";
			this->label_ERR->Size = System::Drawing::Size(85, 45);
			this->label_ERR->TabIndex = 25;
			this->label_ERR->Text = L"ERR";
			this->label_ERR->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_INY
			// 
			this->label_INY->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_INY->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_INY->Location = System::Drawing::Point(396, 58);
			this->label_INY->Name = L"label_INY";
			this->label_INY->Size = System::Drawing::Size(85, 45);
			this->label_INY->TabIndex = 26;
			this->label_INY->Text = L"INY";
			this->label_INY->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_INZ
			// 
			this->label_INZ->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_INZ->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_INZ->Location = System::Drawing::Point(481, 58);
			this->label_INZ->Name = L"label_INZ";
			this->label_INZ->Size = System::Drawing::Size(85, 45);
			this->label_INZ->TabIndex = 27;
			this->label_INZ->Text = L"INZ";
			this->label_INZ->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_NY
			// 
			this->label_NY->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_NY->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_NY->Location = System::Drawing::Point(651, 58);
			this->label_NY->Name = L"label_NY";
			this->label_NY->Size = System::Drawing::Size(85, 45);
			this->label_NY->TabIndex = 28;
			this->label_NY->Text = L"NY";
			this->label_NY->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_NX
			// 
			this->label_NX->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_NX->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_NX->Location = System::Drawing::Point(566, 58);
			this->label_NX->Name = L"label_NX";
			this->label_NX->Size = System::Drawing::Size(85, 45);
			this->label_NX->TabIndex = 29;
			this->label_NX->Text = L"NX";
			this->label_NX->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_PZ
			// 
			this->label_PZ->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_PZ->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_PZ->Location = System::Drawing::Point(736, 13);
			this->label_PZ->Name = L"label_PZ";
			this->label_PZ->Size = System::Drawing::Size(85, 45);
			this->label_PZ->TabIndex = 30;
			this->label_PZ->Text = L"PZ";
			this->label_PZ->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_NZ
			// 
			this->label_NZ->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_NZ->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_NZ->Location = System::Drawing::Point(736, 58);
			this->label_NZ->Name = L"label_NZ";
			this->label_NZ->Size = System::Drawing::Size(85, 45);
			this->label_NZ->TabIndex = 31;
			this->label_NZ->Text = L"NZ";
			this->label_NZ->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_PY
			// 
			this->label_PY->BackColor = System::Drawing::SystemColors::ControlDark;
			this->label_PY->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label_PY->Location = System::Drawing::Point(651, 13);
			this->label_PY->Name = L"label_PY";
			this->label_PY->Size = System::Drawing::Size(85, 45);
			this->label_PY->TabIndex = 32;
			this->label_PY->Text = L"PY";
			this->label_PY->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_Date
			// 
			this->label_Date->BackColor = System::Drawing::Color::Bisque;
			this->label_Date->BorderStyle = System::Windows::Forms::BorderStyle::Fixed3D;
			this->label_Date->Location = System::Drawing::Point(843, 13);
			this->label_Date->Name = L"label_Date";
			this->label_Date->Size = System::Drawing::Size(150, 45);
			this->label_Date->TabIndex = 33;
			this->label_Date->Text = L"2013/MM/DD";
			this->label_Date->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label_Time
			// 
			this->label_Time->BackColor = System::Drawing::Color::Bisque;
			this->label_Time->BorderStyle = System::Windows::Forms::BorderStyle::Fixed3D;
			this->label_Time->Location = System::Drawing::Point(843, 58);
			this->label_Time->Name = L"label_Time";
			this->label_Time->Size = System::Drawing::Size(150, 45);
			this->label_Time->TabIndex = 34;
			this->label_Time->Text = L"HH:MM";
			this->label_Time->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// pictureBox1
			// 
			this->pictureBox1->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"pictureBox1.Image")));
			this->pictureBox1->Location = System::Drawing::Point(7, 13);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(120, 120);
			this->pictureBox1->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBox1->TabIndex = 35;
			this->pictureBox1->TabStop = false;
			// 
			// timer_RealTimer
			// 
			this->timer_RealTimer->Interval = 20;
			this->timer_RealTimer->Tick += gcnew System::EventHandler(this, &Form1::timer_RealTimer_Tick);
			// 
			// label_Speed
			// 
			this->label_Speed->AutoSize = true;
			this->label_Speed->Location = System::Drawing::Point(138, 115);
			this->label_Speed->Name = L"label_Speed";
			this->label_Speed->Size = System::Drawing::Size(109, 18);
			this->label_Speed->TabIndex = 36;
			this->label_Speed->Text = L"NowSpeed:";
			// 
			// button_DEGRAD
			// 
			this->button_DEGRAD->Font = (gcnew System::Drawing::Font(L"Verdana", 9, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button_DEGRAD->Location = System::Drawing::Point(134, 11);
			this->button_DEGRAD->Name = L"button_DEGRAD";
			this->button_DEGRAD->Size = System::Drawing::Size(68, 21);
			this->button_DEGRAD->TabIndex = 6;
			this->button_DEGRAD->Text = L"DEG";
			this->button_DEGRAD->UseVisualStyleBackColor = true;
			this->button_DEGRAD->Click += gcnew System::EventHandler(this, &Form1::button_DEGRAD_Click);
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(11, 18);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->BackColor = System::Drawing::SystemColors::ButtonHighlight;
			this->ClientSize = System::Drawing::Size(1008, 730);
			this->Controls->Add(this->label_Speed);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->label_Time);
			this->Controls->Add(this->label_Date);
			this->Controls->Add(this->label_PY);
			this->Controls->Add(this->label_NZ);
			this->Controls->Add(this->label_PZ);
			this->Controls->Add(this->label_NX);
			this->Controls->Add(this->label_NY);
			this->Controls->Add(this->label_INZ);
			this->Controls->Add(this->label_INY);
			this->Controls->Add(this->label_ERR);
			this->Controls->Add(this->label_INX);
			this->Controls->Add(this->label_M30);
			this->Controls->Add(this->label_HZ);
			this->Controls->Add(this->label_PX);
			this->Controls->Add(this->label_HX);
			this->Controls->Add(this->label_HY);
			this->Controls->Add(this->label_PGM);
			this->Controls->Add(this->label_M00);
			this->Controls->Add(this->textBox_filePath);
			this->Controls->Add(this->button_OpenFile);
			this->Controls->Add(this->richTextBox_OpenFileContent);
			this->Controls->Add(this->groupBox6);
			this->Controls->Add(this->groupBox5);
			this->Controls->Add(this->groupBox4);
			this->Controls->Add(this->groupBox3);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->button_WHEELMode);
			this->Controls->Add(this->button_EmergencyStop);
			this->Controls->Add(this->button_CoordinateChange);
			this->Controls->Add(this->button_TEACHMode);
			this->Controls->Add(this->button_HOME);
			this->Controls->Add(this->button_JOGMode);
			this->Controls->Add(this->button_MDIMode);
			this->Controls->Add(this->button_MEMMode);
			this->Font = (gcnew System::Drawing::Font(L"Verdana", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->Margin = System::Windows::Forms::Padding(6, 4, 6, 4);
			this->Name = L"Form1";
			this->Text = L"Mini_CNC";
			this->FormClosed += gcnew System::Windows::Forms::FormClosedEventHandler(this, &Form1::Form1_FormClosed);
			this->Load += gcnew System::EventHandler(this, &Form1::Form1_Load);
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar_OverRide))->EndInit();
			this->groupBox2->ResumeLayout(false);
			this->groupBox3->ResumeLayout(false);
			this->groupBox4->ResumeLayout(false);
			this->groupBox5->ResumeLayout(false);
			this->groupBox6->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
    /////////Form_Load
	private: System::Void Form1_Load(System::Object^  sender, System::EventArgs^  e) 
			 {
			   cncDataPtr = &cncData;
               

			   cncDataPtr->linkData.motionCtl.singleBlock = false;
			   cncDataPtr->linkData.motionCtl.singleBlockTrigger = true;

			   int i ;
			   
               Init();



			   cncInit(&cncData);
	

			   // init axisData
			   for ( i = 0 ; i < 3 ; i++ )
			   {
				   axisData[i].mSetValue		= axisInitValue[i];
				   axisData[i].mActValue		= axisInitValue[i];

				   axisData[i].remainValue = 0.0;
			   }

			   // resolution of U axis
			   axisData[0].resolution = 0.0018;		// 360(degree) / 200000(increment)
			   // resolution of V axis
			   axisData[1].resolution = 0.0018;
			   // resolution of W axis
			   axisData[2].resolution = 0.0025;		// 200(mm) / 80000(increment)



			   // init motion card and card timer
#if HD_TIMER
			   // init motion card
			   initMotionCard(axisInitValue[0], axisInitValue[1], axisInitValue[2],REAL_SAMPLING_TIME,Timer_ISR_Function);
			   // enable timer
			   enableHDTimer(REAL_SAMPLING_TIME);	
			   timer_RealTimer->Enabled		= false;
			   timer_NonRealTimer->Enabled	= true;
			   linkDataPtr->timeInterval = REAL_SAMPLING_TIME;
			   intpDataPtr->TimeInterval = linkDataPtr->timeInterval;
#else
			   timer_RealTimer->Interval = REAL_SAMPLING_TIME;
			   timer_RealTimer->Enabled		= true;
			   timer_NonRealTimer->Enabled	= true;
			   linkDataPtr->timeInterval = timer_RealTimer->Interval;
			   intpDataPtr->TimeInterval = linkDataPtr->timeInterval;
#endif

			   
			   
			 }
/// Form Close
private: System::Void Form1_FormClosed(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e) 
		 {
			 // close motion card and card timer
#if HD_TIMER
			 disableHDTimer();
			 closeMotionCard();
#endif
		 }

    /////////OpenFile  
	private: System::Void button_OpenFile_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				 openFileDialog->InitialDirectory = "";
				 openFileDialog->Filter = "文字檔(*.txt)|*.txt|所有檔案(*.*)|*.*";
				 openFileDialog->FilterIndex = 2;
				 openFileDialog->DefaultExt = "*.txt";
				 openFileDialog->FileName = "";
				 openFileDialog->RestoreDirectory = true;

				 if( openFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK )
				 {
				
					 textBox_filePath->Text = openFileDialog->FileName;
					 richTextBox_OpenFileContent->LoadFile( openFileDialog->FileName, RichTextBoxStreamType::PlainText);
					 strcpy (filePath,(char*)(void*)Marshal::StringToHGlobalAnsi( openFileDialog->FileName ) );
                
				 }
			 }

    /////////Track Bar Scroll
	private: System::Void trackBar_OverRide_Scroll(System::Object^  sender, System::EventArgs^  e) 
			 {
				 label_FeedOverRide->Text = trackBar_OverRide->Value.ToString() + "%";
			 }

    /////////Initial Statement 
    void Init()
			 {
				 label_FeedOverRide->Text = "100%";
				 OpMode = HOME;
				 button_HOME->ForeColor = Color::White;
				 button_OpenFile -> Enabled = false;
				 CoordinateState = 1;
				 DegRad = 0; 
				 CoordinateChange = false;
			 }

	void StateOperator()
	{
		linkDataPtr->formToSys.OpMode = OpMode;

		this->label_XCoordinateValue->Text = linkDataPtr->intpToMmi.xyzPos[0].ToString("0.000");
		this->label_YCoordinateValue->Text = linkDataPtr->intpToMmi.xyzPos[1].ToString("0.000");
		this->label_ZCoordinateValue->Text = linkDataPtr->intpToMmi.xyzPos[2].ToString("0.000");

		if(DegRad == 0)
		{
		this->label_UCoordinateValue->Text = linkDataPtr->intpToMmi.uvwPos[0].ToString("0.000");
		this->label_VCoordinateValue->Text = linkDataPtr->intpToMmi.uvwPos[1].ToString("0.000");
		this->label_WCoordinateValue->Text = linkDataPtr->intpToMmi.uvwPos[2].ToString("0.000");
		}
		else if(DegRad == 1)
		{
			this->label_UCoordinateValue->Text = (linkDataPtr->intpToMmi.uvwPos[0]*180/PI).ToString("0.000");
			this->label_VCoordinateValue->Text = (linkDataPtr->intpToMmi.uvwPos[1]*180/PI).ToString("0.000");
			this->label_WCoordinateValue->Text = linkDataPtr->intpToMmi.uvwPos[2].ToString("0.000"); 
		}
	
		label_Speed->Text = "Now Speed:"+linkDataPtr->intpToMmi.NowSpeed.ToString("0.000");

		if( linkDataPtr->intpToMmi.xyzPos[0] >= 150.0 )
		{
			label_NX->BackColor = Color::DarkGray;
			label_PX->BackColor = Color::PaleGreen;
		}
		else if (linkDataPtr->intpToMmi.xyzPos[0] <= -150)
		{
			label_NX->BackColor = Color::PaleGreen;
			label_PX->BackColor = Color::DarkGray;
		}

		if( linkDataPtr->intpToMmi.xyzPos[1] >= 150.0 )
		{
			label_NY->BackColor = Color::DarkGray;
			label_PY->BackColor = Color::PaleGreen;
		}
		else if (linkDataPtr->intpToMmi.xyzPos[1] <= -150)
		{
			label_NY->BackColor = Color::PaleGreen;
			label_PY->BackColor = Color::DarkGray;
		}	

		if( linkDataPtr->intpToMmi.xyzPos[2] >= 100.0 )
		{
			label_NZ->BackColor = Color::DarkGray;
			label_PZ->BackColor = Color::PaleGreen;
		}
		else if (linkDataPtr->intpToMmi.xyzPos[2] <= 0)
		{
			label_PZ->BackColor = Color::DarkGray;
			label_NZ->BackColor = Color::PaleGreen;
		}

		if(linkDataPtr->decToMmi.StateERR)
		{
			switch(linkDataPtr->decToMmi.ERRType)
			{
			case ERR_MAIN:
				 label_ERR->Text = "ERR_M";
				 label_ERR->BackColor = Color::Red;
				 break;
			case ERR_WRITE:
				label_ERR->Text = "ERR_W";
				label_ERR->BackColor = Color::Red;
				break;
			case ERR_N:
				label_ERR->Text = "ERR_N";
				label_ERR->BackColor = Color::Red;
				break;
			case ERR_F:
				label_ERR->Text = "ERR_F";
				label_ERR->BackColor = Color::Red;
				break;
			case ERR_G:
				label_ERR->Text = "ERR_G";
				label_ERR->BackColor = Color::Red;
				break;
			case ERR_M:
				label_ERR->Text = "ERR_M30";
				label_ERR->BackColor = Color::Red;
				break;
			default:
				break;
			}
		}

		else
		{
			  label_ERR->Text = "NERR";
			  label_ERR->BackColor = Color::LightBlue;
		}
	}
    


	/////////Tag:XYZ UVW +-
	private: System::Void button_XUCoordinatePlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
            if((OpMode == JOG || OpMode == WHEEL || OpMode == TEACH) )
			{
				 xyzButton[0] = 1;
			     button_XUCoordinatePlus->BackColor = Color::BlueViolet;
				 CoordinateChange = false;
			}
			 }
	private: System::Void button_XUCoordinateMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
			if((OpMode == JOG || OpMode == WHEEL || OpMode == TEACH) )
			{
				 xyzButton[0] = -1;
				 button_XUCoordinateMinus->BackColor = Color::BlueViolet;
				  CoordinateChange = false;
			} 
			 }
	private: System::Void button_YVCoordinatePlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
			if((OpMode == JOG || OpMode == WHEEL || OpMode == TEACH) )
			{
				 xyzButton[1] = 1; 
				 button_YVCoordinatePlus->BackColor = Color::BlueViolet;
				  CoordinateChange = false;
			} 
			}
	private: System::Void button_YVCoordinateMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
            if((OpMode == JOG || OpMode == WHEEL || OpMode == TEACH) )
			{
				 xyzButton[1] = -1;
				 button_YVCoordinateMinus->BackColor = Color::BlueViolet;
				  CoordinateChange = false;
			} 
			 }
	private: System::Void button_ZWCoordinatePlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
		    if((OpMode == JOG || OpMode == WHEEL || OpMode == TEACH)  )
			{
				 xyzButton[2] = 1;
				 button_ZWCoordinatePlus->BackColor = Color::BlueViolet;
				  CoordinateChange = false;
		    }
			 }
	private: System::Void button_ZWCoordinateMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
			if((OpMode == JOG || OpMode == WHEEL || OpMode == TEACH)  )
			{
				 xyzButton[2] = -1;
				 button_ZWCoordinateMinus->BackColor = Color::BlueViolet;
				  CoordinateChange = false;
			} 
			 }

	private: System::Void button_XUCoordinatePlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
			if(OpMode == JOG || OpMode == WHEEL || OpMode == TEACH)
			{
				 xyzButton[0] = 0;
				 button_XUCoordinatePlus->BackColor = Color::WhiteSmoke;
				  CoordinateChange = false;
			} 
			 }
	private: System::Void button_XUCoordinateMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
			if(OpMode == JOG || OpMode == WHEEL || OpMode == TEACH)
			{
				 xyzButton[0] = 0;
				 button_XUCoordinateMinus->BackColor = Color::WhiteSmoke;
				  CoordinateChange = false;
			} 
			 }
	private: System::Void button_YVCoordinatePlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
			if(OpMode == JOG || OpMode == WHEEL || OpMode == TEACH)
			{
				 xyzButton[1] = 0;
				 button_YVCoordinatePlus->BackColor = Color::WhiteSmoke;
				  CoordinateChange = false;
			} 
			 }
	private: System::Void button_YVCoordinateMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
		    if(OpMode == JOG || OpMode == WHEEL || OpMode == TEACH)
			{
	             xyzButton[1] = 0;
				 button_YVCoordinateMinus->BackColor = Color::WhiteSmoke;
				  CoordinateChange = false;
			} 
			 }
	private: System::Void button_ZWCoordinatePlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
			if(OpMode == JOG || OpMode == WHEEL || OpMode == TEACH)
			{
				 xyzButton[2] = 0;
				 button_ZWCoordinatePlus->BackColor = Color::WhiteSmoke;
				  CoordinateChange = false;
			} 
			 }
	private: System::Void button_ZWCoordinateMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
			if(OpMode == JOG || OpMode == WHEEL || OpMode == TEACH)
			{
				 xyzButton[2] = 0;
				 button_ZWCoordinateMinus->BackColor = Color::WhiteSmoke;
				  CoordinateChange = false;
			} 
			}



	/////////Tag:Bottom Function Bar 		   
    /////////MEMMode Button
 	private: System::Void button_MEMMode_Click(System::Object^  sender, System::EventArgs^  e) 
			 {

                    OpMode = MEM;
					button_MEMMode -> ForeColor = Color::White;
					button_OpenFile -> Enabled = true;
					button_MDIMode -> ForeColor = Color::Black;
					button_JOGMode -> ForeColor = Color::Black;
					button_WHEELMode -> ForeColor = Color::Black;
					button_HOME -> ForeColor = Color::Black;
					button_TEACHMode -> ForeColor = Color::Black;
					button_TeachIn -> Enabled = false;
					button_Record -> Enabled = false;
					button_TeachEnd -> Enabled = false;

					richTextBox_OpenFileContent->ReadOnly = true;
					//formReset();
			 }

    /////////MDIMode Button
	private: System::Void button_MDIMode_Click(System::Object^  sender, System::EventArgs^  e)
			 {

                    OpMode = MDI;
					button_MDIMode -> ForeColor = Color::White;
					button_OpenFile -> Enabled = false;
					button_MEMMode -> ForeColor = Color::Black;
					button_JOGMode -> ForeColor = Color::Black;
					button_WHEELMode -> ForeColor = Color::Black;
					button_HOME -> ForeColor = Color::Black;
					button_TEACHMode -> ForeColor = Color::Black;
					button_TeachIn -> Enabled = false;
					button_Record -> Enabled = false;
					button_TeachEnd -> Enabled = false;

					richTextBox_OpenFileContent->ReadOnly =false;

					//formReset();
                    
			 }

    /////////JOGMode Button
	private: System::Void button_JOGMode_Click(System::Object^  sender, System::EventArgs^  e) 
			 {

                    OpMode = JOG;
					button_JOGMode -> ForeColor = Color::White;
					button_OpenFile -> Enabled = false;
					button_MEMMode -> ForeColor = Color::Black;
					button_MDIMode -> ForeColor = Color::Black;
					button_WHEELMode -> ForeColor = Color::Black;
					button_HOME -> ForeColor = Color::Black;
					button_TEACHMode -> ForeColor = Color::Black;
					linkDataPtr->mmiToSys.wheelMulti = 1;
					richTextBox_OpenFileContent->ReadOnly =  true;
					button_TeachIn -> Enabled = false;
					button_Record -> Enabled = false;
					button_TeachEnd -> Enabled = false;
					//formReset();
			 }

    /////////HOME Button
	private: System::Void button_HOME_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
                    OpMode = HOME;
					button_HOME -> ForeColor = Color::White;
					button_OpenFile -> Enabled = false;
					button_MEMMode -> ForeColor = Color::Black;
					button_MDIMode -> ForeColor = Color::Black;
					button_WHEELMode -> ForeColor = Color::Black;
					button_JOGMode -> ForeColor = Color::Black;
					button_TEACHMode -> ForeColor = Color::Black;
					button_TeachIn -> Enabled = false;
					button_Record -> Enabled = false;
					button_TeachEnd -> Enabled = false;

					richTextBox_OpenFileContent->ReadOnly =  true;
					//formReset();
			 }

    /////////WHEELMode Button
	private: System::Void button_WHEELMode_Click(System::Object^  sender, System::EventArgs^  e)
			 {
				    OpMode = WHEEL;
					button_WHEELMode -> ForeColor = Color::White;
					button_OpenFile -> Enabled = false;
					button_MEMMode -> ForeColor = Color::Black;
					button_MDIMode -> ForeColor = Color::Black;
					button_JOGMode -> ForeColor = Color::Black;
					button_HOME -> ForeColor = Color::Black;
					button_TEACHMode -> ForeColor = Color::Black;
					button_TeachIn -> Enabled = false;
					button_Record -> Enabled = false;
					button_TeachEnd -> Enabled = false;

					richTextBox_OpenFileContent->ReadOnly = true;
					//formReset();
			 }

    /////////TEACHMode Button
	private: System::Void button_TEACHMode_Click(System::Object^  sender, System::EventArgs^  e) 
			 {

				    OpMode = TEACH;
					linkDataPtr->mmiToSys.wheelMulti = 1;
					button_TEACHMode -> ForeColor = Color::White;
					button_OpenFile -> Enabled = false;
					button_MEMMode -> ForeColor = Color::Black;
					button_MDIMode -> ForeColor = Color::Black;
					button_JOGMode -> ForeColor = Color::Black;
					button_HOME -> ForeColor = Color::Black;
					button_WHEELMode -> ForeColor = Color::Black;
					button_TeachIn -> Enabled = true;
					button_Record -> Enabled = true;
					button_TeachEnd -> Enabled = true;

					richTextBox_OpenFileContent->ReadOnly = true;
					//formReset();
			 }

    /////////Coordinate Change
	private: System::Void button_CoordinateChange_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
                    if ( CoordinateState == 1 )
					{
						CoordinateState = 0;
						button_CoordinateChange->Text = "XYZ";
						button_XUCoordinatePlus->Text = "U+";
						button_XUCoordinateMinus->Text = "U-";
						button_YVCoordinatePlus->Text = "V+";
						button_YVCoordinateMinus->Text = "V-";
						button_ZWCoordinatePlus->Text = "W+";
						button_ZWCoordinateMinus->Text = "W-";
						CoordinateChange = true;
					}

					else
					{
	                    CoordinateState = 1;
						button_CoordinateChange->Text = "UVW";
						button_XUCoordinatePlus->Text = "X+";
						button_XUCoordinateMinus->Text = "X-";
						button_YVCoordinatePlus->Text = "Y+";
						button_YVCoordinateMinus->Text = "Y-";
						button_ZWCoordinatePlus->Text = "Z+";
						button_ZWCoordinateMinus->Text = "Z-";
						CoordinateChange = true;
					}
			 }

    /////////EMG Stop
	private: System::Void button_EmergencyStop_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				
			 }

	/////////WHEEL Mode Function Button
	private: System::Void button_Wheel_X1_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				 if(OpMode == WHEEL || OpMode == TEACH)
				 {
					 linkDataPtr->mmiToSys.wheelMulti = 1;
					 button_Wheel_X1->BackColor = Color::LightSkyBlue;
					 button_Wheel_X10->BackColor = Color::WhiteSmoke;
					 button_Wheel_X100->BackColor = Color::WhiteSmoke;					 
				 }
			 }

	private: System::Void button_Wheel_X10_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				 if(OpMode == WHEEL || OpMode == TEACH)
				 {
					  linkDataPtr->mmiToSys.wheelMulti = 2;
					  button_Wheel_X1->BackColor = Color::WhiteSmoke;
					  button_Wheel_X10->BackColor = Color::LightSkyBlue;
					  button_Wheel_X100->BackColor = Color::WhiteSmoke;	
				 }
			 }

	private: System::Void button_Wheel_X100_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				 if(OpMode == WHEEL || OpMode == TEACH)
				 {
					 linkDataPtr->mmiToSys.wheelMulti = 3;
					 button_Wheel_X1->BackColor = Color::WhiteSmoke;
					 button_Wheel_X10->BackColor = Color::WhiteSmoke;
					 button_Wheel_X100->BackColor = Color::LightSkyBlue;	
				 }
			 }
    /////////NRT control
	private: System::Void timer_NonRealTimer_Tick(System::Object^  sender, System::EventArgs^  e) 
			 {
				#if HD_TIMER
				 label_UCoordinateValue->Text = axisData[0].mActValue.ToString("0.000"); //徑度
				 label_VCoordinateValue->Text = axisData[1].mActValue.ToString("0.000");
				 label_WCoordinateValue->Text = axisData[2].mActValue.ToString("0.000");
				 cncNrtCtl(&cncData);
				 StateOperator();
				 linkDataPtr->formToSys.OpMode = OpMode;
				 TeachCtl(TeachDataPtr);
				#else
				 cncNrtCtl(&cncData);
				 linkDataPtr->formToSys.OpMode = OpMode;
				 TeachCtl(TeachDataPtr);
				#endif
			 }
    /////////Real Timer
	private: System::Void timer_RealTimer_Tick(System::Object^  sender, System::EventArgs^  e) 
			 {
				 cncRtCtl(&cncData);
				 if((OpMode == JOG || OpMode == WHEEL || OpMode == TEACH) && CoordinateState == 0)
				 {
				 linkDataPtr->intpToMmi.uvwPos[0] += 0.25 * xyzButton[0] * linkDataPtr->mmiToSys.wheelMulti * PI/180;
				 linkDataPtr->intpToMmi.uvwPos[1] += 0.25 * xyzButton[1] * linkDataPtr->mmiToSys.wheelMulti * PI /180;
				 linkDataPtr->intpToMmi.uvwPos[2] += 0.15 * xyzButton[2] * linkDataPtr->mmiToSys.wheelMulti;
				 
				  if (!CoordinateChange)
				 trans(linkDataPtr->intpToMmi.xyzPos,linkDataPtr->intpToMmi.uvwPos);

				 

				 for(int q = 0 ; q < 3 ; q++)
				 {
					 cncDataPtr->intpData.XYZ_Start[q] = linkDataPtr->intpToMmi.xyzPos[q];
		             cncDataPtr->intpData.UVWOld[q] = linkDataPtr->intpToMmi.uvwPos[q];
				 }

				 }	

				 else if ( (OpMode == JOG  || OpMode == WHEEL || OpMode == TEACH ) && CoordinateState == 1)
				 {

					 linkDataPtr->intpToMmi.xyzPos[0] += 0.05 * xyzButton[0] * linkDataPtr->mmiToSys.wheelMulti;
					 linkDataPtr->intpToMmi.xyzPos[1] += 0.05 * xyzButton[1] * linkDataPtr->mmiToSys.wheelMulti;
					 linkDataPtr->intpToMmi.xyzPos[2] += 0.15 * xyzButton[2] * linkDataPtr->mmiToSys.wheelMulti;
					 cncDataPtr->intpData.nowPosition[0] = linkDataPtr->intpToMmi.xyzPos[0];
					 cncDataPtr->intpData.nowPosition[1] = linkDataPtr->intpToMmi.xyzPos[1];
					 cncDataPtr->intpData.nowPosition[2] = linkDataPtr->intpToMmi.xyzPos[2];

					 if (!CoordinateChange)
					 backTransform(&cncDataPtr->intpData);

					 linkDataPtr->intpToMmi.uvwPos[0] = cncDataPtr->intpData.nowUVW[0];
                     linkDataPtr->intpToMmi.uvwPos[1] = cncDataPtr->intpData.nowUVW[1];
                     linkDataPtr->intpToMmi.uvwPos[2] = cncDataPtr->intpData.nowUVW[2];

					 for(int q = 0 ; q < 3 ; q++)
					 {
						 cncDataPtr->intpData.XYZ_Start[q] = linkDataPtr->intpToMmi.xyzPos[q];
						 cncDataPtr->intpData.UVWOld[q] = cncDataPtr->intpData.nowUVW[q];
					 }


				 }
//				 this->label_Time->Text = counter.ToString("0");

                 StateOperator();

			 }
private: System::Void button_FH_Click(System::Object^  sender, System::EventArgs^  e)
		 {

			 if(cncDataPtr->linkData.motionCtl.feedHold == true)
			 {
                 cncDataPtr->linkData.motionCtl.feedHold = false;
				 button_FH->BackColor = Color::WhiteSmoke;
			 }
			 else
			 {
				 cncDataPtr->linkData.motionCtl.feedHold = true;
				 button_FH->BackColor = Color::LightCoral;
			 }
		 }
private: System::Void button_OverRideStart_Click(System::Object^  sender, System::EventArgs^  e) 
		 {

		 if(OpMode == MDI)
		 {

			 if(richTextBox_OpenFileContent->TextLength > 0)
				 mmiDataPtr->ncFileValid = true;

		 }


        if(OpMode == MEM || OpMode == MDI)
		{
	    	if(linkDataPtr->intpToMmi.programEnd)
			{
              if(OpMode == MEM)
			  {
				mmiDataPtr->ncFileValid = true;			
				strcpy(mmiDataPtr->ncFileName , (char*)(void*)Marshal::StringToHGlobalAnsi(openFileDialog->FileName));   
			  }
			  else if (OpMode == MDI)
			  {
				  mmiDataPtr->TextBoxPtr = new wchar_t[richTextBox_OpenFileContent->TextLength];
				  mmiDataPtr->MDINCLength = richTextBox_OpenFileContent->TextLength;

				  for (int i=0 ; i<richTextBox_OpenFileContent->TextLength; i++)
				  {
					  mmiDataPtr->TextBoxPtr[i] = richTextBox_OpenFileContent->Text[i];
				  }			

			  }

			  }

			  if(cncDataPtr->linkData.motionCtl.singleBlock)
			   cncDataPtr->linkData.motionCtl.singleBlockTrigger = true;

		}



		if(OpMode == HOME)
		{
			mmiDataPtr->ncFileValid = true;
		    char HomeName[12]={"HOME.txt"};
			strcpy(mmiDataPtr->ncFileName, HomeName);

		}

		if(linkDataPtr->mmiToSys.M01switch)
		{
			linkDataPtr->motionCtl.singleBlockTrigger = true;
		}

		 }
private: System::Void button_OverRideReset_Click(System::Object^  sender, System::EventArgs^  e) 
		 {
			cncInit(cncDataPtr);
		 }
private: System::Void button_SB_Click(System::Object^  sender, System::EventArgs^  e) 
		 {
			 if(cncDataPtr->linkData.motionCtl.singleBlock == true)
				{
				cncDataPtr->linkData.motionCtl.singleBlock = false;
				button_SB->BackColor = Color::WhiteSmoke;
			    }

			 else
			   {
				cncDataPtr->linkData.motionCtl.singleBlock = true;
				button_SB->BackColor = Color::LightSalmon;
			   }

		 }
private: System::Void button_M01_Click(System::Object^  sender, System::EventArgs^  e) 
		 {
			 if(linkDataPtr->mmiToSys.M01switch)
			 {
				 linkDataPtr->mmiToSys.M01switch = false;
				 button_M01->BackColor = Color::WhiteSmoke;
			 }
			 else
			 {
				 linkDataPtr->mmiToSys.M01switch = true;
				 button_M01->BackColor = Color::LightCoral;
			 }

		 }
private: System::Void button_TeachIn_Click(System::Object^  sender, System::EventArgs^  e) 
		 {
             TeachDataPtr->TeachState = TEACHIN;
		 }
private: System::Void button_Record_Click(System::Object^  sender, System::EventArgs^  e) 
		 {
			 TeachDataPtr->TeachState = RECORD;
		 }
private: System::Void button_TeachEnd_Click(System::Object^  sender, System::EventArgs^  e) 
		 {
			 TeachDataPtr->TeachState = TEACHEND;
			System::Windows::Forms::MessageBox::Show("Teach File Write OK!");
		 }
private: System::Void button_DEGRAD_Click(System::Object^  sender, System::EventArgs^  e) 
		 {
			 if (DegRad == 1)
			 {
				 DegRad = 0;
				 button_DEGRAD->Text = "DEG";
			 }

			 else if (DegRad == 0)
			 {
				 DegRad = 1;
				 button_DEGRAD->Text = "RAD";
			 }

		 }
};



}

