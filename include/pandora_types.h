#ifndef __PANDORA_TYPES_H
#define __PANDORA_TYPES_H


typedef struct PandarPacket_s
{
  double stamp;
  uint8_t data[1240];
}PandarPacket;

typedef struct GPS_STRUCT_{
    int used;
    time_t gps;
}GPS_STRUCT_T;

#endif