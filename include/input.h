#ifndef __PANDAR_INPUT_H
#define __PANDAR_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include "pandora_types.h"

namespace pandar_pointcloud
{

class Input
{
public:
  Input(uint16_t port);
  virtual ~Input() {}
  /** @brief Read one pandar packet.
   *
   * @param pkt points to pandarPacket message
   *
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int getPacket(PandarPacket *pkt,
                        const double timeOffset) = 0;

protected:
  uint16_t port_;
  std::string devip_str_;
};

class InputSocket:public Input
{
public:
  InputSocket(uint16_t port);
  virtual ~InputSocket();

  virtual int getPacket(PandarPacket *pkt,
                        const double timeOffset);
  void setDeviceIP(const std::string &ip);

private:
  int sockfd_;
  in_addr devip_;
  struct timeval getPacketStartTime, getPacketStopTime;
};

} // pandar_driver namespace

#endif // __PANDAR_INPUT_H
