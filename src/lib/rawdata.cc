/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Pandar40 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Pandar40 LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Pandar40 data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Yang Sheng
 *
 */

#include <fstream>
#include <math.h>
#include "angles.cc"
#include "rawdata.h"

namespace pandar_rawdata
{

static double block_offset[BLOCKS_PER_PACKET];
static double laser_offset[LASER_COUNT];

////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData(std::string& correctionFile)
{
    config_.calibrationFile = correctionFile;
    config_.min_angle = 0;
    config_.max_angle = 36000;
    config_.min_range = 0.5;
    config_.max_range = 250.0;

    bufferPacket = new raw_packet_t[1000];
    bufferPacketSize = 0;
    lastBlockEnd = 0;
    lastTimestamp = 0;

    block_offset[5] = 55.1f * 0.0 + 45.18f;
    block_offset[4] = 55.1f * 1.0 + 45.18f;
    block_offset[3] = 55.1f * 2.0 + 45.18f;
    block_offset[2] = 55.1f * 3.0 + 45.18f;
    block_offset[1] = 55.1f * 4.0 + 45.18f;
    block_offset[0] = 55.1f * 5.0 + 45.18f;

    laser_offset[3] = 0.93f * 1.0f;
    laser_offset[35] = 0.93f * 2.0f;
    laser_offset[39] = 0.93f * 3.0f;
    laser_offset[23] = 0.93f * 3.0f + 1.6f * 1.0f;
    laser_offset[16] = 0.93f * 3.0f + 1.6f * 2.0f;
    laser_offset[27] = 0.93f * 4.0f + 1.6f * 2.0f;
    laser_offset[11] = 0.93f * 4.0f + 1.6f * 3.0f;
    laser_offset[31] = 0.93f * 5.0f + 1.6f * 3.0f;
    laser_offset[28] = 0.93f * 6.0f + 1.6f * 3.0f;
    laser_offset[15] = 0.93f * 6.0f + 1.6f * 4.0f;
    laser_offset[2] = 0.93f * 7.0f + 1.6f * 4.0f;
    laser_offset[34] = 0.93f * 8.0f + 1.6f * 4.0f;
    laser_offset[38] = 0.93f * 9.0f + 1.6f * 4.0f;
    laser_offset[20] = 0.93f * 9.0f + 1.6f * 5.0f;
    laser_offset[13] = 0.93f * 9.0f + 1.6f * 6.0f;
    laser_offset[24] = 0.93f * 9.0f + 1.6f * 7.0f;
    laser_offset[8] = 0.93f * 9.0f + 1.6f * 8.0f;
    laser_offset[30] = 0.93f * 10.0f + 1.6f * 8.0f;
    laser_offset[25] = 0.93f * 11.0f + 1.6f * 8.0f;
    laser_offset[12] = 0.93f * 11.0f + 1.6f * 9.0f;
    laser_offset[1] = 0.93f * 12.0f + 1.6f * 9.0f;
    laser_offset[33] = 0.93f * 13.0f + 1.6f * 9.0f;
    laser_offset[37] = 0.93f * 14.0f + 1.6f * 9.0f;
    laser_offset[17] = 0.93f * 14.0f + 1.6f * 10.0f;
    laser_offset[10] = 0.93f * 14.0f + 1.6f * 11.0f;
    laser_offset[21] = 0.93f * 14.0f + 1.6f * 12.0f;
    laser_offset[5] = 0.93f * 14.0f + 1.6f * 13.0f;
    laser_offset[29] = 0.93f * 15.0f + 1.6f * 13.0f;
    laser_offset[22] = 0.93f * 15.0f + 1.6f * 14.0f;
    laser_offset[9] = 0.93f * 15.0f + 1.6f * 15.0f;
    laser_offset[0] = 0.93f * 16.0f + 1.6f * 15.0f;
    laser_offset[32] = 0.93f * 17.0f + 1.6f * 15.0f;
    laser_offset[36] = 0.93f * 18.0f + 1.6f * 15.0f;
    laser_offset[14] = 0.93f * 18.0f + 1.6f * 16.0f;
    laser_offset[7] = 0.93f * 18.0f + 1.6f * 17.0f;
    laser_offset[18] = 0.93f * 18.0f + 1.6f * 18.0f;
    laser_offset[4] = 0.93f * 19.0f + 1.6f * 18.0f;
    laser_offset[26] = 0.93f * 20.0f + 1.6f * 18.0f;
    laser_offset[19] = 0.93f * 20.0f + 1.6f * 19.0f;
    laser_offset[6] = 0.93f * 20.0f + 1.6f * 20.0f;
    

}

/** Update parameters: conversions and update */
void RawData::setParameters(double min_range,
                            double max_range,
                            double view_direction,
                            double view_width)
{
    config_.min_range = min_range;
    config_.max_range = max_range;

	//TODO: YS: not support view angle setting currently.
    //converting angle parameters into the pandar reference (rad)
    config_.tmp_min_angle = view_direction + view_width/2;
    config_.tmp_max_angle = view_direction - view_width/2;

    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);

    //converting into the hardware pandar ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion
    config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle)
    {
        //avoid returning empty cloud if min_angle = max_angle
        config_.min_angle = 0;
        config_.max_angle = 36000;
    }
}

/** Set up for on-line operation. */
int RawData::setup()
{
    printf("rawdata setup\n");
    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
        printf("Unable to open calibration file: %s", config_.calibrationFile.c_str());
        return -1;
    }

    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
        float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
        // printf("rotation: %d, %f, %f\n", rot_index, rotation, cosf(rotation));
        cos_lookup_table_[rot_index] = cosf(rotation);
        sin_lookup_table_[rot_index] = sinf(rotation);
    }
    return 0;
}

int RawData::parseRawData(raw_packet_t* packet, const uint8_t* buf, const int len)
{
    if(len != PACKET_SIZE) {
		printf("packet size mismatch!\n");
        return -1;
	}

    int index = 0;
    // 6x BLOCKs
    for(int i = 0 ; i < BLOCKS_PER_PACKET ; i++) {
		raw_block_t& block = packet->blocks[i];
        block.sob = (buf[index] & 0xff)| ((buf[index + 1] & 0xff)<< 8);
        block.azimuth = (buf[index + 2]& 0xff) | ((buf[index + 3]& 0xff) << 8);
        index += SOB_ANGLE_SIZE;
        // 40x measures
        for(int j = 0 ; j < LASER_COUNT ; j++) {
			raw_measure_t& measure = block.measures[j];
            measure.range = (buf[index]& 0xff)
				| ((buf[index + 1]& 0xff) << 8)
				| ((buf[index + 2]& 0xff) << 16 );
            // printf("%d, %d, %d\n", buf[index], buf[index + 1], buf[index + 2]);
            // printf("parseRawData measure.range: %d, %d\n", j, measure.range);
            measure.reflectivity = (buf[index + 3]& 0xff)
				| ((buf[index + 4]& 0xff) << 8);

            // TODO: Filtering wrong data for LiDAR Bugs.
            if((measure.range == 0x010101 && measure.reflectivity == 0x0101)
					|| measure.range > (200 * 1000 /2 /* 200m -> 2mm */)) {
                measure.range = 0;
                measure.reflectivity = 0;
            }
            index += RAW_MEASURE_SIZE;
        }
    }

    index += RESERVE_SIZE; // skip reserved bytes

    packet->revolution = (buf[index]& 0xff)| (buf[index + 1]& 0xff) << 8;
    index += REVOLUTION_SIZE;

    packet->timestamp = (buf[index]& 0xff)| (buf[index + 1]& 0xff) << 8 |
                        ((buf[index + 2 ]& 0xff) << 16) | ((buf[index + 3]& 0xff) << 24);
    index += TIMESTAMP_SIZE;
    packet->factory[0] = buf[index]& 0xff;
    packet->factory[1] = buf[index + 1]& 0xff;
    index += FACTORY_ID_SIZE;
    return 0;
}

void RawData::computeXYZIR(PPoint& point, int azimuth,
		const raw_measure_t& laserReturn, const pandar_pointcloud::PandarLaserCorrection& correction)
{
    double cos_azimuth, sin_azimuth;
    // printf("computerXYZIR laserRetrun range: %d\n", laserReturn.range);
    double distanceM = laserReturn.range * 0.002;

    point.intensity = static_cast<float> (laserReturn.reflectivity >> 8);
    if (distanceM < config_.min_range || distanceM > config_.max_range)
    {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
        return;
    }
    if (correction.azimuthCorrection == 0)
    {
        cos_azimuth = cos_lookup_table_[azimuth];
        sin_azimuth = sin_lookup_table_[azimuth];
    }
    else
    {
        double azimuthInRadians = angles::from_degrees( (static_cast<double> (azimuth) / 100.0) + correction.azimuthCorrection);
        cos_azimuth = std::cos (azimuthInRadians);
        sin_azimuth = std::sin (azimuthInRadians);
    }

    distanceM += correction.distanceCorrection;
    // printf("distanceM: %f\n", distanceM);
    // printf("correction.cosVertCorrection: %f\n", correction.cosVertCorrection);
    double xyDistance = distanceM * correction.cosVertCorrection;

    point.x = static_cast<float> (xyDistance * sin_azimuth - correction.horizontalOffsetCorrection * cos_azimuth);
    point.y = static_cast<float> (xyDistance * cos_azimuth + correction.horizontalOffsetCorrection * sin_azimuth);
    point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.verticalOffsetCorrection);

    // float a = point.x;
    // point.x = - point.y;
    // point.y = a;

    if (point.x == 0 && point.y == 0 && point.z == 0)
    {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
    }
}

static int PandarEnableList[LASER_COUNT] = {
	1,
	1,
	1,
    1,
	1,
    1,
	1,
    1,
	1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
};

void RawData::toPointClouds (raw_packet_t* packet, PPointCloud& pc)
{
    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
		const raw_block_t& firing_data = packet->blocks[i];

        for (int j = 0; j < LASER_COUNT; j++) {
	    if(PandarEnableList[j] != 1)
		continue;
            PPoint xyzir;
            computeXYZIR (xyzir, firing_data.azimuth,
					firing_data.measures[j], calibration_.laser_corrections[j]);
            if (pcl_isnan (xyzir.x) || pcl_isnan (xyzir.y) || pcl_isnan (xyzir.z))
            {
                continue;
            }
			// xyzir.ring = j;
			pc.points.push_back(xyzir);
			pc.width++;
        }
    }
}

// void RawData::toPointClouds (raw_packet_t* packet, PPointCloud& pc)
// {
//     for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
//         const raw_block_t& firing_data = packet->blocks[i];

//         for (int j = 0; j < LASER_COUNT; j++) {
//         if(PandarEnableList[j] != 1)
//         continue;
//             PPoint xyzir;
//             computeXYZIR (xyzir, firing_data.azimuth,
//                     firing_data.measures[j], calibration_.laser_corrections[j]);
//             if (pcl_isnan (xyzir.x) || pcl_isnan (xyzir.y) || pcl_isnan (xyzir.z))
//             {
//                 continue;
//             }
//             xyzir.ring = j;
//             pc.points.push_back(xyzir);
//             pc.width++;
//         }
//     }
// }

void RawData::toPointClouds (raw_packet_t* packet,int block ,  PPointCloud& pc , double stamp , double& firstStamp)
{
    int first = 0;
    const raw_block_t& firing_data = packet->blocks[block];
    for (int i = 0; i < LASER_COUNT; i++) {
        // if(i == 0)
        // {
        //     double cur_time = stamp - ((double)(block_offset[block] + laser_offset[i])/1000000.0f);
        //     // double cur_time = stamp - ((block_offset[block] + laser_offset[i])/1000000);
        //     double diff = packet->recv_time - cur_time;
        //     if(diff < 0.0f)
        //     {
        //         ROS_ERROR("ERROR TIME %lf %lf %f " , cur_time , packet->recv_time , diff);
        //     }
        // }
            PPoint xyzir;
            // printf("firing_data.measures[i]: %f\n", firing_data.measures[i]);
            computeXYZIR (xyzir, firing_data.azimuth,
                    firing_data.measures[i], calibration_.laser_corrections[i]);
            if (pcl_isnan (xyzir.x) || pcl_isnan (xyzir.y) || pcl_isnan (xyzir.z))
            {
                continue;
            }

            xyzir.timestamp = stamp - ((double)(block_offset[block] + laser_offset[i])/1000000.0f);
            if(!first)
            {
                // ROS_ERROR("%f" , xyzir.timestamp);
                firstStamp = xyzir.timestamp;
                first = 1;
            }
            
            xyzir.ring = i;
            pc.points.push_back(xyzir);
            pc.width++;
    }
}

void RawData::toPointClouds (raw_packet_t* packet,int laser , int block,  PPointCloud& pc)
{
    int i = block;
    {
        const raw_block_t& firing_data = packet->blocks[i];
            PPoint xyzir;
            computeXYZIR (xyzir, firing_data.azimuth,
                    firing_data.measures[laser], calibration_.laser_corrections[laser]);
            if (pcl_isnan (xyzir.x) || pcl_isnan (xyzir.y) || pcl_isnan (xyzir.z))
            {
                return;
            }
            // xyzir.ring = laser;
            pc.points.push_back(xyzir);
            pc.width++;
    }
}

int RawData::unpack(PandarPacket &packet, PPointCloud &pc, time_t& gps1 , 
                                            gps_struct_t &gps2 , double& firstStamp, int& lidarRotationStartAngle)
{
    currentPacketStart = bufferPacketSize == 0 ? 0 :bufferPacketSize -1 ;
    // for(int mi = 0; mi < 1240; ++mi)
    // {
    //     printf("%d, %d\n", mi, packet.data[mi]);
    // }
    // return 0;
    parseRawData(&bufferPacket[bufferPacketSize++], &packet.data[0], 1240);

    int hasAframe = 0;
    int currentBlockEnd = 0;
    int currentPacketEnd = 0;
    if(bufferPacketSize > 1)
    {
        int lastAzumith = -1;
        for(int i = currentPacketStart ; i < bufferPacketSize ; i++)
        {
            if(hasAframe)
            {
                break;
            }

            int j = 0;
            if (i == currentPacketStart)
            {
                /* code */
                j = lastBlockEnd;
            }
            else
            {
                j = 0;
            }
            for (; j < BLOCKS_PER_PACKET; ++j)
            {
                /* code */
                if(lastAzumith == -1)
                {
                    lastAzumith = bufferPacket[i].blocks[j].azimuth;
                    continue;
                }


                if(lastAzumith > bufferPacket[i].blocks[j].azimuth)
                {
                    if (lidarRotationStartAngle <= bufferPacket[i].blocks[j].azimuth)
                    {
                        // ROS_ERROR("rotation, %d, %d, %d", lastAzumith, bufferPacket[i].blocks[j].azimuth, lidarRotationStartAngle);
                        currentBlockEnd = j;
                        hasAframe = 1;
                        currentPacketEnd = i;
                        break;
                    }

                }
                else if (lastAzumith < lidarRotationStartAngle && lidarRotationStartAngle <= bufferPacket[i].blocks[j].azimuth)
                {
                    // ROS_ERROR("%d, %d, %d", lastAzumith, bufferPacket[i].blocks[j].azimuth, lidarRotationStartAngle);
                    currentBlockEnd = j;
                    hasAframe = 1;
                    currentPacketEnd = i;
                    break;
                }
                lastAzumith = bufferPacket[i].blocks[j].azimuth;
            }
        }
    }

    if(hasAframe)
    {
        int first = 0;
        int j = 0;
        for (int k = 0; k < (currentPacketEnd + 1); ++k)
        {
            if(k == 0)
                j = lastBlockEnd;
            else
                j = 0;

            

            // if > 500ms 
            if(bufferPacket[k].timestamp < 500000 && gps2.used == 0)
            {
                if(gps1 > gps2.gps)
                {
                    printf("Oops , You give me a wrong timestamp I think...");
                }
                gps1 = gps2.gps;
                gps2.used =1;
            }
            else
            {
                if(bufferPacket[k].timestamp < lastTimestamp)
                {
                    int gap = (int)lastTimestamp - (int)bufferPacket[k].timestamp;
                    // avoid the fake jump... wrong udp order
                    if(gap > (10 * 1000)) // 10ms
                    {
                        // Oh , there is a round. But gps2 is not changed , So there is no gps packet!!!
                        // We need to add the offset.
                        
                        gps1 += ((lastTimestamp-20) /1000000) +  1; // 20us offset , avoid the timestamp of 1000002...
                        printf("There is a round , But gps packet!!! , Change gps1 by manual!!! %d %d %d " , gps1 , lastTimestamp , bufferPacket[k].timestamp);
                    }
                    
                }
            }
            int timestamp = bufferPacket[k].timestamp;


            // int gap = timestamp - lastTimestamp;
            // gap = gap <0 ? gap + 1000000 : gap;
            // if(gap > 600)
            // {
            //     ROS_ERROR("gap too large %d-%d=%d"  , timestamp ,lastTimestamp , gap);
            // }
            // ROS_ERROR("timestamp : %d %lf %d"  , timestamp ,ros::Time::now().toSec() , gps1);
            // ROS_ERROR("timestamp of this packe t : %lf" ,(double)gps1 + (((double)bufferPacket[k].timestamp)/1000000));
            for (; j < BLOCKS_PER_PACKET; ++j)
            {
                /* code */
                if (currentBlockEnd == j && k == (currentPacketEnd))
                {
                    break;
                }
                double stamp = 0.0;
                toPointClouds(&bufferPacket[k] , j, pc ,(double)gps1 + (((double)bufferPacket[k].timestamp)/1000000) , stamp);
                if(!first && stamp != 0.0)
                {
                    firstStamp = stamp;
                    first = 1;
                }
                
            } 
            lastTimestamp = bufferPacket[k].timestamp;
        }
        memcpy(&bufferPacket[0] , &bufferPacket[currentPacketEnd] , sizeof(raw_packet_t) * (bufferPacketSize - currentPacketEnd));
        bufferPacketSize = bufferPacketSize - currentPacketEnd;
        lastBlockEnd = currentBlockEnd;
        return 1;
    }
    else
    {
        return 0;
    }
}

} // namespace pandar_rawdata
