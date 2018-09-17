#include <windows.h>
#include <iostream>
#include <iomanip>
#include "marvelmind.h"
#include "Header.h"
#include <vector>
#include <math.h>

using namespace DimensionEngineering;
struct PositionValue MowerPosition;
struct StatBeaconPosition STB;

int main() {
	std::cout << "Starting System";
	double Spacing =3;
	struct MarvelmindHedge * hedge = createMarvelmindHedge();
	
	bool Value = false;

	int i = 1;
	std::vector<double> Xvalues;
	std::vector<double> Yvalues;
	int temp = 0;

	struct UsbSabertoothSerial ^ Connection = gcnew UsbSabertoothSerial("COM4", 9600);
	struct UsbSabertooth ^ ST = gcnew UsbSabertooth(Connection);
	double ChangeX, Power;
	startMarvelmindHedge(hedge);
	while (Value == false) {
		Value = NavigationPoints(hedge, Spacing, Xvalues, Yvalues);
	}
	std::cout << "Y Max: " << STB.MaxY << "  Y Min: " << STB.MinY;
	int temp2 = 0;
	while (i < (5))
	{
		
		double SlopeX;
		double StoreX;
		Sleep(100);
		ST->Motor(1, 400);
		ST->Motor(2, 400);
		getPositionFromMarvelmindHedge(hedge, &MowerPosition);
		int x = MowerPosition.x; 
		SlopeX = x - StoreX;
		if(x == 0)
		{
			continue;
		}
		std::cout << "\nDesired X: " << Xvalues[i] << "  Actual X: " << MowerPosition.x / 1000;
		std::cout << "\nY Maximum: " << STB.MaxY << "  Actual Y: " << MowerPosition.y / 1000;
		std::cout << "\n Slope:  " << SlopeX;
		
		
		
		
		// Going UP
		if (temp == 1)
		{
			
			if ((MowerPosition.y/1000) < STB.MinY)
			{
				std::cout << "\nTurn left";
				ST->Motor(1, 0);
				ST->Motor(2, 400);
				Sleep(2000);
				ST->Motor(1, -400);
				ST->Motor(2, 400);
				Sleep(1200);
				ST->Motor(1, 0);
				ST->Motor(2, 0);
				Sleep(250);
				ST->Motor(1, 400);
				ST->Motor(2, 400);
				Sleep(200);
			
				temp = 0;
				i++;
				StoreX = 0;
				continue;
			}
			if ((MowerPosition.x / 1000) < (Xvalues[i] + .1*Xvalues[i]) && (MowerPosition.x / 1000) > (Xvalues[i] - .1*Xvalues[i]))
			{
				if (temp2 == 1)
				{
					ST->Motor(1, 375);
					ST->Motor(2, 450);
					Sleep(200);
				}
				ST->Motor(1, 400);
				ST->Motor(2, 400);
				std::cout << "\nGo Straight...";
				temp2 = 0;
				continue;
			}
			if ((MowerPosition.x/1000) > Xvalues[i])
			{
				StoreX = MowerPosition.x;
				std::cout << "\nVeer Right";
				ST->Motor(1, 400);
				ST->Motor(2, 350);
				Sleep(100);
				temp2 = 1;

			}
			else if ((MowerPosition.x/1000) < Xvalues[i])
			{
				StoreX = MowerPosition.x; 
				std::cout << "\nVeer Left";
				ST->Motor(1, 300);
				ST->Motor(2, 400);
				Sleep(100);
				temp2 = 1;
		
			}
		}
		else // Going Down
		{
			if ((MowerPosition.y/1000) > STB.MaxY)
			{

				std::cout << "\nTurn right";
				ST->Motor(1, 400);
				ST->Motor(2, 0);
				Sleep(2000);
				ST->Motor(1, 300);
				ST->Motor(2, - 400);
				Sleep(1200);
				ST->Motor(1, 0);
				ST->Motor(2, 0);
				Sleep(250);
				ST->Motor(1, 400);
				ST->Motor(2, 400);
				Sleep(200);
				temp = 1;
				i++;
				StoreX = 0;
				continue;
			}
			if ((MowerPosition.x / 1000) < (Xvalues[i] + .1*Xvalues[i]) && (MowerPosition.x / 1000) > (Xvalues[i] - .1*Xvalues[i]))
			{
				if (temp2 == 1)
				{
					ST->Motor(1, 450);
					ST->Motor(2, 375);
					Sleep(200);
				}
				ST->Motor(1, 400);
				ST->Motor(2, 400);
				std::cout << "\nGo Straight...";
				temp2 = 0;
				continue;
			}
			if ((MowerPosition.x/1000) < Xvalues[i])
			{
				StoreX = MowerPosition.x;
				std::cout << "\nVeer Right";
				ST->Motor(1, 400);
				ST->Motor(2, 350);
				temp2 = 1;


			}
			else if ((MowerPosition.x/1000) > Xvalues[i])
			{
				StoreX = MowerPosition.x;
				std::cout << "\nVeer Left";
				ST->Motor(1, 350);
				ST->Motor(2, 400);
				temp2 = 1;

			}
		}

	}
	ST->Motor(1.0);
	ST->Motor(2, 0);
	std::cin.get();
	stopMarvelmindHedge(hedge);
	destroyMarvelmindHedge(hedge);

}



bool NavigationPoints(struct MarvelmindHedge * hedge, double Spacing, std::vector<double> &Xvalues, std::vector<double> &Yvalues) {
	double xm[5], ym[5];
	struct StationaryBeaconsPositions positions;
	bool Value;

	std::cout << "\nInitilalizing";
	for (int z = 1; z < 5; z++) {

		getStationaryBeaconsPositionsFromMarvelmindHedge(hedge, &positions);
		Sleep(300);

		struct StationaryBeaconPosition *b;

		for (int i = 0; i < 5; i++)
		{
			b = &positions.beacons[i];

			xm[i] = ((double)b->x) / 1000.0;
			ym[i] = ((double)b->y) / 1000.0;

		}
		std::cout << ".";
	}
	std::cout << "";

	for (int i = 0; i < 4; i++)
	{
		STB.SBP[i][0] = xm[i];
		STB.SBP[i][1] = ym[i];
	}


	Value = GenerateMap(Spacing, Xvalues, Yvalues);

	return Value;

}


bool GenerateMap(double Spacing, std::vector<double> &Xvalues, std::vector<double> &Yvalues)
{
	double tempMax = 0;
	double tempMin = 0;
	double xValues;
	double yValues;

	// Finding Maximum X values
	for (int i = 0; i<4; i++)
	{
		if (STB.SBP[i][0] > tempMax)
			tempMax = STB.SBP[i][0];
		
	}
	STB.MaxX = tempMax;
	// Finding Minimum X values
	for (int i = 0; i<4; i++)
	{
		if (STB.SBP[i][0] < tempMin)
			tempMin = STB.SBP[i][0];
	}
	STB.MinX = tempMin;
	// Determining The Number of Points to Make sure that the Marvelmind Stationary beacons constructed Correctly
	double NumPoints = (tempMax - tempMin) / Spacing;
	if (NumPoints > 200) {
		std::cout << " failed... Retrying...";
		return false;
	}

	// Displaying X Values


	xValues = tempMin;
	for (int i = 0; i <= (NumPoints - 2); i++)
	{

		xValues = xValues + Spacing;
		Xvalues.push_back(xValues);

	}

	double tempMaxY;
	double tempMinY;
	// Finding Maximum Y Values
	for (int i = 0; i<4; i++)
	{
		if (STB.SBP[i][1] > tempMaxY)
			tempMaxY = STB.SBP[i][1];
	}
	STB.MaxY = tempMaxY;
	// Finding Minimum Y Values
	for (int i = 0; i<4; i++)
	{
		if (STB.SBP[i][1] < tempMinY)
			tempMinY = STB.SBP[i][1];
	}
	STB.MaxX = tempMinY;


	// Determining The Number of Points to Make sure that the Marvelmind Stationary beacons constructed Correctly
	NumPoints = (tempMaxY - tempMinY) / (Spacing);
	if (NumPoints > 200) {
		std::cout << "\nInitialization failed... Retrying...";
		return false;
	}

	// Displaying Y Values
	yValues = tempMinY;

	for (int i = 0; i <= (NumPoints - 2); i++)
	{
		yValues = yValues + Spacing;
		Yvalues.push_back(yValues);

	}
	std::vector< std::vector<double>> Ret;
	Ret = { { Xvalues },{ Yvalues } };

	return true;
}