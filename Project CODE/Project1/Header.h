
#include <windows.h>
#include <iostream>
#include "marvelmind.h"
#include <vector>
using namespace DimensionEngineering;


struct StatBeaconPosition {
	double SBP[4][2];
	double MotorValues[2];
	double MaxY;
	double MaxX;
	double MinX;
	double MinY;
};


bool GenerateMap(double Spacing, std::vector<double> &Xvalues, std::vector<double> &Yvalues);

bool NavigationPoints(struct MarvelmindHedge *hedge, double Spacing, std::vector<double> &Xvalues, std::vector<double> &Yvalues);

double RunMotors(struct MarvelmindHedge *hedge, std::vector<double> &Xvalues, std::vector<double> &Yvalues, int i, int j, UsbSabertooth ^ST, int NewValues);

void GetMowerDirection(struct MarvelmindHedge *hedge, UsbSabertooth ^ST);