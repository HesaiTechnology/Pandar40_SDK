#include <fstream>
#include <math.h>
#include "rawdata.h"

namespace pandar_rawdata
{

static double blockOffset[BLOCKS_PER_PACKET];
static double laserOffset[LASER_COUNT];

RawData::RawData(const std::string &correctionFile)
{
  config_.calibrationFile = correctionFile;
  config_.minRange = PandoraSDK_MIN_RANGE;
  config_.maxRange = PandoraSDK_MAX_RANGE;

  bufferPacket = new RAW_PACKET_T[1000];
  bufferPacketSize = 0;
  lastBlockEnd = 0;
  lastTimestamp = 0;

  blockOffset[5] = 55.1f * 0.0 + 45.18f;
  blockOffset[4] = 55.1f * 1.0 + 45.18f;
  blockOffset[3] = 55.1f * 2.0 + 45.18f;
  blockOffset[2] = 55.1f * 3.0 + 45.18f;
  blockOffset[1] = 55.1f * 4.0 + 45.18f;
  blockOffset[0] = 55.1f * 5.0 + 45.18f;

  laserOffset[3] = 0.93f * 1.0f;
  laserOffset[35] = 0.93f * 2.0f;
  laserOffset[39] = 0.93f * 3.0f;
  laserOffset[23] = 0.93f * 3.0f + 1.6f * 1.0f;
  laserOffset[16] = 0.93f * 3.0f + 1.6f * 2.0f;
  laserOffset[27] = 0.93f * 4.0f + 1.6f * 2.0f;
  laserOffset[11] = 0.93f * 4.0f + 1.6f * 3.0f;
  laserOffset[31] = 0.93f * 5.0f + 1.6f * 3.0f;
  laserOffset[28] = 0.93f * 6.0f + 1.6f * 3.0f;
  laserOffset[15] = 0.93f * 6.0f + 1.6f * 4.0f;
  laserOffset[2] = 0.93f * 7.0f + 1.6f * 4.0f;
  laserOffset[34] = 0.93f * 8.0f + 1.6f * 4.0f;
  laserOffset[38] = 0.93f * 9.0f + 1.6f * 4.0f;
  laserOffset[20] = 0.93f * 9.0f + 1.6f * 5.0f;
  laserOffset[13] = 0.93f * 9.0f + 1.6f * 6.0f;
  laserOffset[24] = 0.93f * 9.0f + 1.6f * 7.0f;
  laserOffset[8] = 0.93f * 9.0f + 1.6f * 8.0f;
  laserOffset[30] = 0.93f * 10.0f + 1.6f * 8.0f;
  laserOffset[25] = 0.93f * 11.0f + 1.6f * 8.0f;
  laserOffset[12] = 0.93f * 11.0f + 1.6f * 9.0f;
  laserOffset[1] = 0.93f * 12.0f + 1.6f * 9.0f;
  laserOffset[33] = 0.93f * 13.0f + 1.6f * 9.0f;
  laserOffset[37] = 0.93f * 14.0f + 1.6f * 9.0f;
  laserOffset[17] = 0.93f * 14.0f + 1.6f * 10.0f;
  laserOffset[10] = 0.93f * 14.0f + 1.6f * 11.0f;
  laserOffset[21] = 0.93f * 14.0f + 1.6f * 12.0f;
  laserOffset[5] = 0.93f * 14.0f + 1.6f * 13.0f;
  laserOffset[29] = 0.93f * 15.0f + 1.6f * 13.0f;
  laserOffset[22] = 0.93f * 15.0f + 1.6f * 14.0f;
  laserOffset[9] = 0.93f * 15.0f + 1.6f * 15.0f;
  laserOffset[0] = 0.93f * 16.0f + 1.6f * 15.0f;
  laserOffset[32] = 0.93f * 17.0f + 1.6f * 15.0f;
  laserOffset[36] = 0.93f * 18.0f + 1.6f * 15.0f;
  laserOffset[14] = 0.93f * 18.0f + 1.6f * 16.0f;
  laserOffset[7] = 0.93f * 18.0f + 1.6f * 17.0f;
  laserOffset[18] = 0.93f * 18.0f + 1.6f * 18.0f;
  laserOffset[4] = 0.93f * 19.0f + 1.6f * 18.0f;
  laserOffset[26] = 0.93f * 20.0f + 1.6f * 18.0f;
  laserOffset[19] = 0.93f * 20.0f + 1.6f * 19.0f;
  laserOffset[6] = 0.93f * 20.0f + 1.6f * 20.0f;
}

int RawData::setup()
{
  calibration_.read(config_.calibrationFile);
  if (!calibration_.initialized)
  {
    printf("Unable to open calibration file: %s", config_.calibrationFile.c_str());
    return -1;
  }

  for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex)
  {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rotIndex);
    cos_lookup_table_[rotIndex] = cosf(rotation);
    sin_lookup_table_[rotIndex] = sinf(rotation);
  }
  return 0;
}

int RawData::parseRawData(RAW_PACKET_T *packet, const uint8_t *buf, const int len)
{
  if (len != PACKET_SIZE)
  {
    printf("packet size mismatch!\n");
    return -1;
  }

  int index = 0;
  // 6 BLOCKs
  for (int i = 0; i < BLOCKS_PER_PACKET; i++)
  {
    RAW_BLOCK_T &block = packet->blocks[i];
    block.sob = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    block.azimuth = (buf[index + 2] & 0xff) | ((buf[index + 3] & 0xff) << 8);
    index += SOB_ANGLE_SIZE;
    // 40x measures
    for (int j = 0; j < LASER_COUNT; j++)
    {
      RAW_MEASURE_T &measure = block.measures[j];
      measure.range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8) | ((buf[index + 2] & 0xff) << 16);
      // printf("%d, %d, %d\n", buf[index], buf[index + 1], buf[index + 2]);
      // printf("parseRawData measure.range: %d, %d\n", j, measure.range);
      measure.reflectivity = (buf[index + 3] & 0xff) | ((buf[index + 4] & 0xff) << 8);

      // TODO: Filtering wrong data for LiDAR Bugs.
      if ((measure.range == 0x010101 && measure.reflectivity == 0x0101) || measure.range > (200 * 1000 / 2 /* 200m -> 2mm */))
      {
        measure.range = 0;
        measure.reflectivity = 0;
      }
      index += RAW_MEASURE_SIZE;
    }
  }

  index += RESERVE_SIZE; // skip reserved bytes

  packet->revolution = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8;
  index += REVOLUTION_SIZE;

  packet->timestamp = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                      ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;
  packet->factory[0] = buf[index] & 0xff;
  packet->factory[1] = buf[index + 1] & 0xff;
  index += FACTORY_ID_SIZE;
  return 0;
}

void RawData::computeXYZIR(
    PPoint &point,
    int azimuth,
    const RAW_MEASURE_T &laserReturn,
    const pandar_pointcloud::PandarLaserCorrection &correction)
{
  double cos_azimuth, sin_azimuth;
  double distanceM = laserReturn.range * 0.002;

  point.intensity = static_cast<float>(laserReturn.reflectivity >> 8);
  if (distanceM < config_.minRange || distanceM > config_.maxRange)
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
    double azimuthInRadians = angles::from_degrees((static_cast<double>(azimuth) / 100.0) + correction.azimuthCorrection);
    cos_azimuth = std::cos(azimuthInRadians);
    sin_azimuth = std::sin(azimuthInRadians);
  }

  distanceM += correction.distanceCorrection;
  double xyDistance = distanceM * correction.cosVertCorrection;

  point.x = static_cast<float>(xyDistance * sin_azimuth - correction.horizontalOffsetCorrection * cos_azimuth);
  point.y = static_cast<float>(xyDistance * cos_azimuth + correction.horizontalOffsetCorrection * sin_azimuth);
  point.z = static_cast<float>(distanceM * correction.sinVertCorrection + correction.verticalOffsetCorrection);

  if (point.x == 0 && point.y == 0 && point.z == 0)
  {
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
  }
}

void RawData::toPointClouds(
    RAW_PACKET_T *packet,
    int block, PPointCloud &pc,
    double stamp,
    double &firstStamp)
{
  int first = 0;
  const RAW_BLOCK_T &firing_data = packet->blocks[block];
  for (int i = 0; i < LASER_COUNT; i++)
  {
    PPoint xyzir;
    computeXYZIR(xyzir, firing_data.azimuth,
                 firing_data.measures[i], calibration_.laserCorrections[i]);
    if (pcl_isnan(xyzir.x) || pcl_isnan(xyzir.y) || pcl_isnan(xyzir.z))
    {
      continue;
    }

    xyzir.timestamp = stamp - ((double)(blockOffset[block] + laserOffset[i]) / 1000000.0f);
    if (!first)
    {
      firstStamp = xyzir.timestamp;
      first = 1;
    }

    xyzir.ring = i;
    pc.points.push_back(xyzir);
    pc.width++;
  }
}

int RawData::unpack(
    PandarPacket &packet,
    PPointCloud &pc,
    time_t &gps1,
    GPS_STRUCT_T &gps2,
    double &firstStamp,
    int &lidarRotationStartAngle)
{
  currentPacketStart = bufferPacketSize == 0 ? 0 : bufferPacketSize - 1;
  parseRawData(&bufferPacket[bufferPacketSize++], &packet.data[0], 1240);

  int hasAframe = 0;
  int currentBlockEnd = 0;
  int currentPacketEnd = 0;
  if (bufferPacketSize > 1)
  {
    int lastAzumith = -1;
    for (int i = currentPacketStart; i < bufferPacketSize; i++)
    {
      if (hasAframe)
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
        if (lastAzumith == -1)
        {
          lastAzumith = bufferPacket[i].blocks[j].azimuth;
          continue;
        }

        if (lastAzumith > bufferPacket[i].blocks[j].azimuth)
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
          currentBlockEnd = j;
          hasAframe = 1;
          currentPacketEnd = i;
          break;
        }
        lastAzumith = bufferPacket[i].blocks[j].azimuth;
      }
    }
  }

  if (hasAframe)
  {
    int first = 0;
    int j = 0;
    for (int k = 0; k < (currentPacketEnd + 1); ++k)
    {
      if (k == 0)
        j = lastBlockEnd;
      else
        j = 0;

      // if > 500ms
      if (bufferPacket[k].timestamp < 500000 && gps2.used == 0)
      {
        if (gps1 > gps2.gps)
        {
          printf("Oops , You give me a wrong timestamp I think...");
        }
        gps1 = gps2.gps;
        gps2.used = 1;
      }
      else
      {
        if (bufferPacket[k].timestamp < lastTimestamp)
        {
          int gap = (int)lastTimestamp - (int)bufferPacket[k].timestamp;
          // avoid the fake jump... wrong udp order
          if (gap > (10 * 1000)) // 10ms
          {
            // Oh , there is a round. But gps2 is not changed , So there is no gps packet!!!
            // We need to add the offset.

            gps1 += ((lastTimestamp - 20) / 1000000) + 1; // 20us offset , avoid the timestamp of 1000002...
            printf("There is a round , But gps packet!!! , Change gps1 by manual!!! %d %d %d ", gps1, lastTimestamp, bufferPacket[k].timestamp);
          }
        }
      }
      int timestamp = bufferPacket[k].timestamp;
      for (; j < BLOCKS_PER_PACKET; ++j)
      {
        /* code */
        if (currentBlockEnd == j && k == (currentPacketEnd))
        {
          break;
        }
        double stamp = 0.0;
        toPointClouds(&bufferPacket[k], j, pc, (double)gps1 + (((double)bufferPacket[k].timestamp) / 1000000), stamp);
        if (!first && stamp != 0.0)
        {
          firstStamp = stamp;
          first = 1;
        }
      }
      lastTimestamp = bufferPacket[k].timestamp;
    }
    memcpy(&bufferPacket[0], &bufferPacket[currentPacketEnd], sizeof(RAW_PACKET_T) * (bufferPacketSize - currentPacketEnd));
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
