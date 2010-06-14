/*
 *  AscTec Autopilot Serial Interface
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

#include "asctec_autopilot/crc16.h"
#include "asctec_autopilot/asctec.h"
#include "asctec_autopilot/telemetry.h"

namespace asctec
{
  SerialInterface::SerialInterface (std::string port, uint32_t speed):serialport_name_ (port), serialport_speed_ (speed)
  {
    struct termios tio;
      status = false;
      serialport_baud_ = bitrate (serialport_speed_);
      ROS_INFO ("Initializing serial port...");

      dev_ = fopen (serialport_name_.c_str (), "w+");
      ROS_ASSERT_MSG (dev_ != NULL, "Failed to open serial port: %s", serialport_name_.c_str ());

      ROS_ASSERT_MSG (tcgetattr (fileno (dev_), &tio) == 0, "Unknown Error: %s", strerror (errno));

      cfsetispeed (&tio, serialport_baud_);
      cfsetospeed (&tio, serialport_baud_);

      tio.c_iflag = 0;
      tio.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
      tio.c_iflag |= IGNBRK;

      tio.c_oflag = 0;
      tio.c_oflag &= ~(OPOST | ONLCR);

      tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
      tio.c_cflag &= ~(PARENB | CRTSCTS | CSTOPB);

      tio.c_lflag = 0;
      tio.c_lflag |= NOFLSH;
      tio.c_lflag &= ~(ISIG | IEXTEN | ICANON | ECHO | ECHOE);

      stall (true);

      fflush (dev_);
      tcflush (fileno (dev_), TCIOFLUSH);

      ROS_ASSERT_MSG (dev_ != NULL, "Could not open serial port %s", serialport_name_.c_str ());
      ROS_INFO ("Successfully connected to %s, Baudrate %d\n", serialport_name_.c_str (), serialport_speed_);
  }

  SerialInterface::~SerialInterface ()
  {
    ROS_DEBUG ("Destroying Serial Interface");
    flush ();
    fclose (dev_);
  }

  void SerialInterface::flush ()
  {
    fflush (dev_);
    tcflush (fileno (dev_), TCIOFLUSH);
  }

  void SerialInterface::drain ()
  {
    ROS_ASSERT_MSG (tcdrain (fileno (dev_)) == 0, "Drain Error: %s", strerror (errno));
  }

  void SerialInterface::stall (bool wait)
  {
    struct termios tio;
    ROS_ASSERT_MSG (tcgetattr (fileno (dev_), &tio) == 0, "Unknown Error: %s", strerror (errno));
    if (wait)
    {
      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 1;
    }
    else
    {
      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 0;
    }
    ROS_ASSERT_MSG (tcsetattr (fileno (dev_), TCSADRAIN, &tio) == 0, "Unknown Error: %s", strerror (errno));
  }

  speed_t SerialInterface::bitrate (int Bitrate)
  {
    switch (Bitrate)
    {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      default:                 // invalid bitrate
        return B0;
    }
  }

  bool SerialInterface::getPacket (char *spacket, unsigned char &packet_type, unsigned short &packet_crc,
                                   unsigned short &packet_size)
  {
    char stoken[4];
    char ssize[2];
    char stype[1];
    char scrc[2];

    int i;

    // get beginning (">*>")
    stoken[3] = '\0';

    stall (true);
    i = fread (stoken, sizeof (char), 3, dev_);
    if (i == 0 || strncmp (stoken, ">*>", 3) != 0)
    {
      ROS_ERROR ("Error Reading Packet Header: %s", strerror (errno));
      ROS_DEBUG ("Read (%d): %s", i, stoken);
      flush ();
      return false;
    }
    ROS_DEBUG ("Packet Header OK");

    // get packet size
    i = fread (ssize, sizeof (char), 2, dev_);
    if (i == 0)
    {
      ROS_ERROR ("Error Reading Packet Size: %s", strerror (errno));
      flush ();
      return false;
    }
    memcpy (&packet_size, ssize, sizeof (packet_size));
    ROS_DEBUG ("Packet size: %d", packet_size);

    // get packet type
    i = fread (stype, sizeof (char), 1, dev_);
    if (i == 0)
      return false;
    memcpy (&packet_type, stype, sizeof (packet_type));
    ROS_DEBUG ("Packet type: %d", packet_type);

    // get packet
    i = fread (spacket, sizeof (char), packet_size, dev_);
    if (i == 0)
      return false;
    ROS_DEBUG ("Packet string: ok");

    // get packet crc
    i = fread (scrc, sizeof (char), sizeof (scrc), dev_);
    if (i == 0)
      return false;
    memcpy (&packet_crc, scrc, sizeof (packet_crc));
    ROS_DEBUG ("Packet crc: %d", packet_crc);

    // get closing ("<#<")
    i = fread (stoken, sizeof (char), 3, dev_);
    if (i == 0 || strncmp (stoken, "<#<", 3) != 0)
    {
      ROS_ERROR ("Error Reading Packet Footer: %s", strerror (errno));
      ROS_DEBUG ("Read (%d): %s", i, stoken);
      flush ();
      return false;
    }
    ROS_DEBUG ("Packet Footer OK");

    return true;
  }

  void SerialInterface::write (char *output, int len)
  {
    int i;
    ROS_DEBUG ("Writing %d element(s): %s", len, output);
    ROS_DEBUG ("dev: %d", (int)dev_);
    flush();
    ROS_DEBUG("FOO");
    i = fwrite (output, sizeof (char), len, dev_);
    if (i != len)
    {
      ROS_ERROR ("Error wrote %d out of %d element(s): %s", i, len, strerror (errno));
      ROS_BREAK ();
    }
  }

  bool SerialInterface::getPackets (Telemetry *telemetry)
  {
    flush();
    ROS_DEBUG ("getPackets");
    char cmd[16];
    char spacket[1024];
    unsigned char packet_type;
    unsigned short packet_crc;
    unsigned short packet_size;
    unsigned int i;


    ROS_INFO ("Packet Request: %04x %d packets", (short) telemetry->requestPackets_.to_ulong (),
              telemetry->requestPackets_.count ());
    sprintf (cmd, ">*>p%c", (short) telemetry->requestPackets_.to_ulong ());
    write (cmd, 6);
    drain ();

    bool result = false;
    for (i = 0; i < telemetry->requestPackets_.count(); i++)
    {
      ROS_DEBUG ("getPacket started");
      bool read_result = getPacket (spacket, packet_type, packet_crc, packet_size);

      if (read_result)
      {
        ROS_DEBUG ("getPacket successful: type = %d, crc = %d", packet_type, packet_crc);

        if (packet_type == Telemetry::PD_LLSTATUS)
        {
          ROS_DEBUG ("Packet type is LL_STATUS");
          memcpy (&telemetry->LL_STATUS_, spacket, packet_size);
          if (crc_valid (packet_crc,&telemetry->LL_STATUS_, sizeof (packet_size))) {
            result = true;
            ROS_DEBUG ("Valid CRC!!");
          }
          telemetry->dumpLL_STATUS();
        }
        else if (packet_type == Telemetry::PD_IMUCALCDATA)
        {
          ROS_DEBUG ("Packet type is IMU_CALCDATA");
          memcpy (&telemetry->IMU_CALCDATA_, spacket, packet_size);
          if (crc_valid (packet_crc,&telemetry->IMU_CALCDATA_, packet_size)) {
            result = true;
            ROS_DEBUG ("Valid CRC!!");
          }
          telemetry->dumpIMU_CALCDATA();
        }
        else
        {
          ROS_ERROR("Packet type is UNKNOWN");
        }
      }
      else
      {
        // failed read
        ROS_ERROR("getPacket failed");
        break;
      }
    }
    stall (false);
    i = fread (spacket, sizeof (char), 1, dev_);
    ROS_ASSERT_MSG (i == 0, "Unexpected Data");
    return result;
  }

}
