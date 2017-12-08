#ifndef __PANDORA_TYPES_H
#define __PANDORA_TYPES_H


typedef struct PandarPacket_s
{
  double stamp;
  uint8_t data[1240];
}PandarPacket;

typedef struct gps_struct{
    int used;
    time_t gps;
}gps_struct_t;

#endif