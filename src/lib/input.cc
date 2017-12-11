#include <sstream>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <string.h>
#include <iostream>
#include "input.h"

namespace pandar_pointcloud
{
static const size_t packetSize = sizeof(PandarPacket().data);

Input::Input(uint16_t port) : port_(port)
{
}

InputSocket::InputSocket(uint16_t port) : Input(port)
{
  devip_str_ = "";
  sockfd_ = -1;
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket"); // TODO: perror errno
    return;
  }

  sockaddr_in myAddress;                    // my address information
  memset(&myAddress, 0, sizeof(myAddress)); // initialize to zeros
  myAddress.sin_family = AF_INET;           // host byte order
  myAddress.sin_port = htons(port);         // port in network byte order
  myAddress.sin_addr.s_addr = INADDR_ANY;   // automatically fill in my IP

  if (bind(sockfd_, (sockaddr *)&myAddress, sizeof(sockaddr)) == -1)
  {
    perror("bind"); // TODO: perror errno
    return;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }
}

InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

// return : 0 - lidar
//          2 - gps
//          1 - error
int InputSocket::getPacket(PandarPacket *pkt, const double timeOffset)
{
  gettimeofday(&getPacketStartTime, NULL);
  int isgps = 0;
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000; // one second (in msec)

  sockaddr_in senderAddress;
  socklen_t senderAddressLen = sizeof(senderAddress);
  int retval = poll(fds, 1, POLL_TIMEOUT);
  if (retval < 0) // poll() error?
  {
    if (errno != EINTR)
      printf("poll() error: %s", strerror(errno));
    return 1;
  }
  if (retval == 0) // poll() timeout?
  {
    perror("Pandar poll() timeout");
    return 1;
  }
  if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
  {
    perror("poll() reports Pandar error");
    return 1;
  }
  senderAddressLen = sizeof(senderAddress);
  // Receive packets that should now be available from the
  // socket using a blocking read.
  ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0],
                            packetSize, 0,
                            (sockaddr *)&senderAddress,
                            &senderAddressLen);

  if (nbytes < 0)
  {
    if (errno != EWOULDBLOCK)
    {
      perror("recvfail");
      return 1;
    }
  }

  if ((size_t)nbytes == 512)
  {
    // GPS
    isgps = 1;
  }

  gettimeofday(&getPacketStopTime, NULL);
  pkt->stamp = static_cast<double>(getPacketStartTime.tv_sec + static_cast<double>(getPacketStartTime.tv_usec) / 1000 + getPacketStopTime.tv_sec + static_cast<double>(getPacketStopTime.tv_usec) / 1000) / 2;
  if (isgps)
  {
    return 2;
  }
  return 0;
}

}
