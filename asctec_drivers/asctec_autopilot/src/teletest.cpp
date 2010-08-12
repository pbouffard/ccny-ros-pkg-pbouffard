#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>

#include <ros/ros.h>

#include "asctec_autopilot/telemetry.h"

int main (int argc, char **argv)
{
  asctec::Telemetry::Telemetry tele;
  ROS_INFO ("Testing...");
  tele.enablePolling (asctec::RequestTypes::IMU_CALCDATA, 1);
  tele.enablePolling (asctec::RequestTypes::IMU_RAWDATA, 2, 0);
  tele.enablePolling (asctec::RequestTypes::LL_STATUS, 2, 1);
  tele.enablePolling (asctec::RequestTypes::RC_DATA, 5);
  ROS_INFO ("Packet codes");
  ROS_INFO ("%0X", tele.REQUEST_BITMASK[asctec::RequestTypes::IMU_CALCDATA]);
  ROS_INFO ("%0X", tele.REQUEST_BITMASK[asctec::RequestTypes::IMU_RAWDATA]);
  ROS_INFO ("%0X", tele.REQUEST_BITMASK[asctec::RequestTypes::LL_STATUS]);
  ROS_INFO ("%0X", tele.REQUEST_BITMASK[asctec::RequestTypes::RC_DATA]);
  for (int i = 0; i < 20; i++)
  {
    tele.buildRequest ();
    ROS_INFO ("Counter: %d", tele.requestCount_);
    ROS_INFO ("Request: %04X", (short) tele.requestPackets_.to_ulong ());
    ROS_INFO ("Packets Requested: %zd", tele.requestPackets_.count ());
    tele.requestPackets_ ^= tele.requestPackets_;
    tele.requestCount_++;
  }
  return 0;
}
