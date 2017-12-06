#include <sstream>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include "input.h"
#include <string.h>

namespace pandar_pointcloud
{
static const size_t packet_size = sizeof(PandarPacket().data);

/** @brief constructor
   *
   *  @param port UDP port number
   */
InputSocket::InputSocket(uint16_t port) : port_(port)
{
  devip_str_ = "";
  sockfd_ = -1;
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket"); // TODO: perror errno
    return;
  }

  sockaddr_in my_addr;                  // my address information
  memset(&my_addr, 0, sizeof(my_addr)); // initialize to zeros
  my_addr.sin_family = AF_INET;         // host byte order
  my_addr.sin_port = htons(port);       // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill in my IP

  if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
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

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

// return : 0 - lidar
//          2 - gps
//          1 - error
/** @brief Get one pandar packet. */
int InputSocket::getPacket(PandarPacket *pkt, const double time_offset)
{
  // double time1 = ros::Time::now().toSec();
  gettimeofday(&start_time_getpacket, NULL);
  int isgps = 0;
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000; // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true)
  {
    // Unfortunately, the Linux kernel recvfrom() implementation
    // uses a non-interruptible sleep() when waiting for data,
    // which would cause this method to hang if the device is not
    // providing data.  We poll() the device first to make sure
    // the recvfrom() will not block.
    //
    // Note, however, that there is a known Linux kernel bug:
    //
    //   Under Linux, select() may report a socket file descriptor
    //   as "ready for reading", while nevertheless a subsequent
    //   read blocks.  This could for example happen when data has
    //   arrived but upon examination has wrong checksum and is
    //   discarded.  There may be other circumstances in which a
    //   file descriptor is spuriously reported as ready.  Thus it
    //   may be safer to use O_NONBLOCK on sockets that should not
    //   block.

    // poll() until input available
    do
    {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0) // poll() error?
      {
        if (errno != EINTR)
          printf("poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0) // poll() timeout?
      {
        printf("Pandar poll() timeout");
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
      {
        perror("poll() reports Pandar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);

    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0],
                              packet_size, 0,
                              (sockaddr *)&sender_address,
                              &sender_address_len);

    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        return 1;
      }
    }
    else if ((size_t)nbytes == packet_size)
    {
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
      else
        break; //done
    }
    else if ((size_t)nbytes == 512)
    {
      // GPS
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
      else
      {
        isgps = 1;
        break; //done
      }
    }

    perror("incomplete Pandar packet read");
  }

  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  // double time2 = ros::Time::now().toSec();
  gettimeofday(&stop_time_getpacket, NULL);
  pkt->stamp = (start_time_getpacket.tv_sec + start_time_getpacket.tv_usec / 1000 + stop_time_getpacket.tv_sec + stop_time_getpacket.tv_usec / 1000) / 2.0 + time_offset;
  if (isgps)
  {
    return 2;
  }
  return 0;
}
}