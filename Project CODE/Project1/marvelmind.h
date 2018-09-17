#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DATA_INPUT_SEMAPHORE "/mm_data_input_semaphore"


#ifdef __cplusplus
extern"C" {
#endif	

	struct PositionValue
	{
		int address;
		uint32_t timestamp;
		double x, y, z;// coordinates in millimeters

		double angle;

		bool highResolution;

		bool ready;
		bool processed;
	};

	struct RawIMUValue
	{
		int16_t acc_x;
		int16_t acc_y;
		int16_t acc_z;

		int16_t gyro_x;
		int16_t gyro_y;
		int16_t gyro_z;

		int16_t compass_x;
		int16_t compass_y;
		int16_t compass_z;

		uint32_t timestamp;

		bool updated;
	};

	struct FusionIMUValue
	{
		int32_t x;
		int32_t y;
		int32_t z;// coordinates in mm

		int16_t qw;
		int16_t qx;
		int16_t qy;
		int16_t qz;// quaternion, normalized to 10000

		int16_t vx;
		int16_t vy;
		int16_t vz;// velocity, mm/s

		int16_t ax;
		int16_t ay;
		int16_t az;// acceleration, mm/s^2

		uint32_t timestamp;

		bool updated;
	};

	struct RawDistanceItem
	{
		uint8_t address_beacon;
		uint32_t distance;// distance, mm
	};
	struct RawDistances
	{
		uint8_t address_hedge;
		struct RawDistanceItem distances[4];

		bool updated;
	};

	struct StationaryBeaconPosition
	{
		uint8_t address;
		int32_t x, y, z;// coordinates in millimeters

		bool highResolution;
	};
#define MAX_STATIONARY_BEACONS 30
	struct StationaryBeaconsPositions
	{
		uint8_t numBeacons;
		struct StationaryBeaconPosition beacons[MAX_STATIONARY_BEACONS];

		bool updated;
	};

#define MAX_BUFFERED_POSITIONS 3
	struct MarvelmindHedge
	{
		// serial port device name (physical or USB/virtual). It should be provided as
		// an argument:
		// /dev/ttyACM0 - typical for Linux / Raspberry Pi
		// /dev/tty.usbmodem1451 - typical for Mac OS X
		const char * ttyFileName;

		// Baud rate. Should be match to baudrate of hedgehog-beacon
		// default: 9600
		uint32_t baudRate;

		// buffer of measurements
		struct PositionValue * positionBuffer;

		struct StationaryBeaconsPositions positionsBeacons;

		struct RawIMUValue rawIMU;
		struct FusionIMUValue fusionIMU;

		struct RawDistances rawDistances;

		// verbose flag which activate console output
		//		default: False
		bool verbose;

		//	pause flag. If True, class would not read serial data
		bool pause;

		//  If True, thread would exit from main loop and stop
		bool terminationRequired;

		//  receiveDataCallback is callback function to recieve data
		void(*receiveDataCallback)(struct PositionValue position);
		void(*anyInputPacketCallback)();

		// private variables
		uint8_t lastValuesCount_;
		uint8_t lastValues_next;
		bool haveNewValues_;
#ifdef WIN32
		HANDLE thread_;
		CRITICAL_SECTION lock_;
#else
		pthread_t thread_;
		pthread_mutex_t lock_;
#endif
	};

#define POSITION_DATAGRAM_ID 0x0001
#define BEACONS_POSITIONS_DATAGRAM_ID 0x0002
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID 0x0012
#define IMU_RAW_DATAGRAM_ID 0x0003
#define BEACON_RAW_DISTANCE_DATAGRAM_ID 0x0004
#define IMU_FUSION_DATAGRAM_ID 0x0005

	struct MarvelmindHedge * createMarvelmindHedge();
	void destroyMarvelmindHedge(struct MarvelmindHedge * hedge);
	void startMarvelmindHedge(struct MarvelmindHedge * hedge);

	void printPositionFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		bool onlyNew);
	bool getPositionFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		struct PositionValue * position);

	bool getStationaryBeaconsPositionsFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		struct StationaryBeaconsPositions * positions);
	void printStationaryBeaconsPositionsFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		bool onlyNew);

	bool getRawDistancesFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		struct RawDistances* rawDistances);
	void printRawDistancesFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		bool onlyNew);

	bool getRawIMUFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		struct RawIMUValue* rawIMU);
	void printRawIMUFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		bool onlyNew);

	bool getFusionIMUFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		struct FusionIMUValue *fusionIMU);
	void printFusionIMUFromMarvelmindHedge(struct MarvelmindHedge * hedge,
		bool onlyNew);
	

	void stopMarvelmindHedge(struct MarvelmindHedge * hedge);

#ifdef WIN32
#define DEFAULT_TTY_FILENAME "\\\\.\\COM3"
#else
#define DEFAULT_TTY_FILENAME "/dev/ttyACM0"
#endif // WIN32

#ifdef __cplusplus
}
#endif