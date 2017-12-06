#ifndef __PANDAR_INPUT_H
#define __PANDAR_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include "pandora_types.h"

namespace pandar_pointcloud
{
  static uint16_t DATA_PORT_NUMBER = 8080;     // default data port
  static uint16_t POSITION_PORT_NUMBER = 8308; // default position port

  // typedef struct PandarPacket
  // {
  //   double stamp;
  //   unsigned char data[1240];
  // }PandarPacket;


  class InputSocket
  {
  public:
    InputSocket(uint16_t port = DATA_PORT_NUMBER);
    virtual ~InputSocket();

    virtual int getPacket(PandarPacket *pkt, 
                          const double time_offset);
    void setDeviceIP( const std::string& ip );

  private:
    int sockfd_;
    uint16_t port_;
    std::string devip_str_;
    in_addr devip_;
    struct timeval start_time_getpacket, stop_time_getpacket;
  };

} // pandar_driver namespace

#endif // __PANDAR_INPUT_H
