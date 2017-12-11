
#ifndef __PANDAR_RAWDATA_H
#define __PANDAR_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>
#include <pcl/point_cloud.h>
#include "point_types.h"
#include "calibration.h"
#include "pandora_types.h"

namespace pandar_rawdata
{

#define SOB_ANGLE_SIZE 4
#define RAW_MEASURE_SIZE 5
#define LASER_COUNT 40
#define BLOCKS_PER_PACKET 6
#define BLOCK_SIZE (RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE)
#define TIMESTAMP_SIZE 4
#define FACTORY_ID_SIZE 2
#define RESERVE_SIZE 8
#define REVOLUTION_SIZE 2
#define INFO_SIZE (TIMESTAMP_SIZE + FACTORY_ID_SIZE + RESERVE_SIZE + REVOLUTION_SIZE)
#define PACKET_SIZE (BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE)
#define ROTATION_MAX_UNITS 36001
#define ROTATION_RESOLUTION 0.01

#define PandoraSDK_MIN_RANGE 0.5
#define PandoraSDK_MAX_RANGE 250.0

typedef struct RAW_MEASURE_{
    uint32_t range;
    uint16_t reflectivity;
} RAW_MEASURE_T;

typedef struct RAW_BLOCK
{
    uint16_t sob;
    uint16_t azimuth;
    RAW_MEASURE_T measures[LASER_COUNT];
} RAW_BLOCK_T;



typedef struct RAW_PACKET
{
    RAW_BLOCK_T blocks[BLOCKS_PER_PACKET];
    uint8_t reserved[RESERVE_SIZE];
    uint16_t revolution;
    uint32_t timestamp;
    uint8_t factory[2];
    double recv_time;
} RAW_PACKET_T;

class RawData
{
public:

    RawData(const std::string& correctionFile);
    ~RawData() {}
    int setup();
    int unpack(
        PandarPacket &packet,
        PPointCloud &pc,
        time_t& gps1 , 
        GPS_STRUCT_T &gps2,
        double& firstStamp,
        int& lidarRotationStartAngle);

private:

    typedef struct {
        std::string calibrationFile;     ///< calibration file name
        double maxRange;                ///< maximum range to publish
        double minRange;                ///< minimum range to publish
        int minAngle;                   ///< minimum angle to publish
        int maxAngle;                   ///< maximum angle to publish
    } CONFIG_T;
    CONFIG_T config_;

    pandar_pointcloud::Calibration calibration_;
    float sin_lookup_table_[ROTATION_MAX_UNITS];
    float cos_lookup_table_[ROTATION_MAX_UNITS];
    RAW_PACKET_T *bufferPacket;
    /** in-line test whether a point is in range */
    bool pointInRange(float range)
    {
        return (range >= config_.minRange
                && range <= config_.maxRange);
    }

	int parseRawData(RAW_PACKET_T* packet, const uint8_t* buf, const int len);
    void toPointClouds (RAW_PACKET_T* packet,int laser,  PPointCloud& pc, double stamp, double& firstStamp);
	void computeXYZIR(
        PPoint& point,
        int azimuth,
		const RAW_MEASURE_T& laserReturn,
		const pandar_pointcloud::PandarLaserCorrection& correction);

    int lastBlockEnd;
    int bufferPacketSize;
    int currentPacketStart;
    int lastTimestamp;
    int lastAzumith;
};

} // namespace pandar_rawdata

#endif // __PANDAR_RAWDATA_H
