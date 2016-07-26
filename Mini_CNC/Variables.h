#include <math.h>

//Coordinate
double XCoordinateValue = 0.0;
double YCoordinateValue = 0.0;
double ZCoordinateValue = 0.0;
double UCoordinateValue = 0.0;
double VCoordinateValue = 0.0;
double WCoordinateValue = 0.0;

//ColorChange
int axisButton[3] = {0};
int axisButtonM[3] = {0};

int CoordinateState = 1; // XYZ = 1 UVW = 0
int DegRad = 0;          // Deg = 1 Rad = 0


bool CoordinateChange;

//OpenFile
char filePath[200];

//Mode Select
int OpMode = 0;

#define MEM 1
#define MDI 2
#define JOG 3
#define WHEEL 4
#define TEACH 5
#define HOME 6
#define PI 3.14159
#define ARM_LENGTH 75

void trans(double xyz[3], double uvw[3])
{
	double	tempUVW[3];
	double	tempXYZ[3];
	int		i;

	for ( i = 0 ; i < 3 ; i++ )
	{
		tempUVW[i] = uvw[i] * PI /180;
	}

	tempXYZ[0] = ARM_LENGTH * ( cos(tempUVW[0]) + cos(tempUVW[0] + tempUVW[1]) );
	tempXYZ[1] = ARM_LENGTH * ( sin(tempUVW[0]) + sin(tempUVW[0] + tempUVW[1]) );

	xyz[0] = tempXYZ[0];
	xyz[1] = tempXYZ[1];
	xyz[2] = uvw[2];
}
