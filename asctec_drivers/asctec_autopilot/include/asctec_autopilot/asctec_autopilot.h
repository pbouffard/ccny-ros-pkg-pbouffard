#ifndef ASCTEC_AUTOPILOT_H
#define ASCTEC_AUTOPILOT_H

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
#include "asctec_autopilot/serial_interface.h"

namespace asctec_autopilot
{


  class AutoPilot
  {
  friend class SerialInterface;
  public:
      AutoPilot ();
     ~AutoPilot ();
    class SerialInterface *serialInterface__;
    void enablePolling (uint16_t request, uint16_t interval);
    void buildRequest();
    void spin (const ros::TimerEvent & e);
    // CONVERSION FACTORS
    static const double PEL_TO_ROS_ANGLE = (1.0 / 1000.0) * 3.14159265 / 180.0;        // converts to rad
    static const double PEL_TO_ROS_ANGVEL = (1.0 / 64.8) * 3.14159265 / 180.0; // convetts to rad/s
    static const double PEL_TO_ROS_ACC = (1.0 / 10000.0) * 9.81;       // converts to m/s^s
    static const double PEL_TO_ROS_HEIGHT = (1.0 / 1000.0);    // converts to m

    // request descriptors
    static const uint16_t REQ_LL_STATUS = 0x0001;
    static const uint16_t REQ_IMU_RAWDATA = 0x0002;
    static const uint16_t REQ_IMU_CALCDATA = 0x0004;
    static const uint16_t REQ_RC_DATA = 0x0008;
    static const uint16_t REQ_CONTROLLER_OUTPUT = 0x0010;
    static const uint16_t REQ_GPS_DATA = 0x0080;
    static const uint16_t REQ_WAYPOINT = 0x0100;
    static const uint16_t REQ_GPS_DATA_ADVANCED = 0x0200;
    static const uint16_t REQ_CAM_DATA = 0x0800;       // FIXME: Does this even work?
    //packet descriptors
    static const uint8_t PD_IMURAWDATA = 0x01;
    static const uint8_t PD_LLSTATUS = 0x02;
    static const uint8_t PD_IMUCALCDATA = 0x03;
    static const uint8_t PD_HLSTATUS = 0x04;
    static const uint8_t PD_DEBUGDATA = 0x05;

    static const uint8_t PD_CTRLOUT = 0x11;
    static const uint8_t PD_FLIGHTPARAMS = 0x12;
    static const uint8_t PD_CTRLCOMMANDS = 0x13;
    static const uint8_t PD_CTRLINTERNAL = 0x14;
    static const uint8_t PD_RCDATA = 0x15;
    static const uint8_t PD_CTRLSTATUS = 0x16;
    static const uint8_t PD_CTRLINPUT = 0x17;
    static const uint8_t PD_CTRLFALCON = 0x18;

    static const uint8_t PD_WAYPOINT = 0x20;
    static const uint8_t PD_CURRENTWAY = 0x21;
    static const uint8_t PD_NMEADATA = 0x22;
    static const uint8_t PD_GPSDATA = 0x23;

    static const uint8_t PD_CAMERACOMMANDS = 0x30;
  private:
    ros::Timer timer_;
    ros::NodeHandle nh_;
    double freq;
/*
  inline
  void updateCtrlChecksum()
  {
  // CTRL_Input.chksum = CTRL_Input.pitch + CTRL_Input.roll + CTRL_Input.yaw + CTRL_Input.thrust + CTRL_Input.ctrl + 0xAAAA;
  // startstring: >*>di
  } 
*/
    // Data Request
    // >*>p[unsigned short packets]








    struct CTRL_INPUT
    {                           //serial commands (= Scientific Interface)
      short pitch;              //Pitch input: -2047..+2047 (0=neutral)
      short roll;               //Roll input: -2047..+2047 (0=neutral)
      short yaw;                //(=R/C Stick input) -2047..+2047 (0=neutral)
      short thrust;             //Collective: 0..4095 = 0..100%
      short ctrl;               /*control byte:
                                   bit 0: pitch control enabled
                                   bit 1: roll control enabled
                                   bit 2: yaw control enabled
                                   bit 3: thrust control enabled
                                   These bits can be used to only enable one axis at a time and thus to control the other axes manually.
                                   This usually helps a lot to set up and finetune controllers for each axis seperately. */
      short chksum;
    };

    struct LL_STATUS
    {
      //battery voltages in mV
      short battery_voltage_1;
      short battery_voltage_2;
      //don’t care
      short status;
      //Controller cycles per second (should be ˜1000)
      short cpu_load;
      //don’t care
      char compass_enabled;
      char chksum_error;
      char flying;
      char motors_on;
      short flightMode;
      //Time motors are turning
      short up_time;
    };

    struct IMU_RAWDATA
    {
      //pressure sensor 24-bit value, not scaled but bias free
      int pressure;
      //16-bit gyro readings; 32768 = 2.5V
      short gyro_x;
      short gyro_y;
      short gyro_z;
      //10-bit magnetic field sensor readings
      short mag_x;
      short mag_y;
      short mag_z;
      //16-bit accelerometer readings
      short acc_x;
      short acc_y;
      short acc_z;
      //16-bit temperature measurement using yaw-gyro internal sensor
      unsigned short temp_gyro;
      //16-bit temperature measurement using ADC internal sensor
      unsigned int temp_ADC;
    };

    struct IMU_CALCDATA
    {
      // angles derived by integration of gyro_outputs, drift compensated by data fusion;
      // -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
      int angle_nick;
      int angle_roll;
      int angle_yaw;
      // angular velocities, raw values [16 bit] but bias free
      int angvel_nick;
      int angvel_roll;
      int angvel_yaw;
      // acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
      short acc_x_calib;
      short acc_y_calib;
      short acc_z_calib;
      // horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
      short acc_x;
      short acc_y;
      short acc_z;
      // reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree
      int acc_angle_nick;
      int acc_angle_roll;
      // total acceleration measured (10000 = 1g)
      int acc_absolute_value;
      // magnetic field sensors output, offset free and scaled;
      // units not determined, as only the direction of the field vector is taken into account
      int Hx;
      int Hy;
      int Hz;

      //compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
      int mag_heading;
      //pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown; used for short-term position stabilization
      int speed_x;
      int speed_y;
      int speed_z;
      //height in mm (after data fusion)
      int height;
      //diff. height in mm/s (after data fusion)
      int dheight;
      //diff. height measured by the pressure sensor [mm/s]
      int dheight_reference;
      //height measured by the pressure sensor [mm]
      int height_reference;
    };

    struct GPS_DATA
    {
      //latitude/longitude in deg * 10ˆ7
      int latitude;
      int longitude;
      //GPS height in mm
      int height;
      //speed in x (E/W) and y(N/S) in mm/s
      int speed_x;
      int speed_y;
      //GPS heading in deg * 1000
      int heading;
      //accuracy estimates in mm and mm/s
      unsigned int horizontal_accuracy;
      unsigned int vertical_accuracy;
      unsigned int speed_accuracy;
      //number of satellite vehicles used in NAV solution
      unsigned int numSV;
      // GPS status information; 0x03 = valid GPS fix
      int status;
    };

    struct GPS_DATA_ADVANCED
    {
      //latitude/longitude in deg * 10ˆ7
      int latitude;
      int longitude;
      //GPS height in mm
      int height;
      //speed in x (E/W) and y(N/S) in mm/s
      int speed_x;
      int speed_y;
      //GPS heading in deg * 1000
      int heading;
      //accuracy estimates in mm and mm/s
      unsigned int horizontal_accuracy;
      unsigned int vertical_accuracy;
      unsigned int speed_accuracy;
      //number of satellite vehicles used in NAV solution
      unsigned int numSV;

      //GPS status information; 0x03 = valid GPS fix
      int status;
      //coordinates of current origin in deg * 10ˆ7
      int latitude_best_estimate;
      int longitude_best_estimate;
      //velocities in X (E/W) and Y (N/S) after data fusion
      int speed_x_best_estimate;
      int speed_y_best_estimate;
    };

    struct RC_DATA
    {
      //channels as read from R/C receiver
      unsigned short channels_in[8];
      //channels bias free, remapped and scaled to 0..4095
      unsigned short channels_out[8];
      //Indicator for valid R/C receiption
      unsigned char lock;
    };

    struct CONTROLLER_OUTPUT
    {
      //attitude controller outputs; 0..200 = -100 ..+100%
      int nick;
      int roll;
      int yaw;
      //current thrust (height controller output); 0..200 = 0..100%
      int thrust;
    };

    struct WAYPOINT
    {
      //always set to 1
      unsigned char wp_number;
      //don't care
      unsigned char dummy_1;
      unsigned short dummy_2;
      //see WPPROP defines below
      unsigned char properties;
      //max. speed to travel to waypoint in % (default 100)
      unsigned char max_speed;
      //time to stay at a waypoint (XYZ) in 1/100th s
      unsigned short time;
      //position accuracy to consider a waypoint reached in mm (default: 2500 (= 2.5 m))
      unsigned short pos_acc;
      //chksum = 0xAAAA + wp.yaw + wp.height + wp.time + wp.X + wp.Y + wp.max_speed + wp.pos_acc + wp.properties + wp.wp_number;
      short chksum;
      //waypoint coordinates in mm // longitude in abs coords
      int X;
      //waypoint coordinates in mm // latitude in abs coords
      int Y;
      //Desired heading at waypoint
      int yaw;
      //height over 0 reference in mm
      int height;
    };

/*

#define WPPROP_ABSCOORDS 0x01 //if set waypoint is interpreted as
absolute coordinates, else relative coords
#define WPPROP_HEIGHTENABLED 0x02 //set new height at waypoint
#define WPPROP_YAWENABLED 0x04 //set new yaw-angle at waypoint
(not yet implemented)
#define WPPROP_AUTOMATICGOTO 0x10 //if set, vehicle will not wait for
a goto command, but goto this waypoint directly
#define WPPROP_CAM_TRIGGER 0x20 //if set, photo camera is triggered
when waypoint is reached and time to stay is 80% up
Sending the waypoint structure to the vehicle:
The following string must be sent to the vehicle, directly followed by the actual waypoint
structure:
unsigned char string[]=">*>ws";
Commands for the waypoint navigation:
>*>wg "Goto waypoint"
>*>wl "Launch / Set Home
>*>we "End flight => land at current position"
>*>wh "Come home"
Sending the launch command when the vehicle is hovering with the switch on the R/C in
"GPS + Height control" sets the home position.
You will receive an acknowledge if a command or a waypoint was received correctly:
>a[1 byte packet descriptor]a<

*/
    struct CMD
    {
      struct CTRL_INPUT CTRL_INPUT_;
    };

      std::bitset < 16 > requestPackets_;
    uint16_t requestCount_;
    uint16_t interval_LL_STATUS_;
    uint16_t offset_LL_STATUS_;
    struct LL_STATUS LL_STATUS_;
    uint16_t interval_IMU_RAWDATA_;
    uint16_t offset_IMU_RAWDATA_;
    struct IMU_RAWDATA IMU_RAWDATA_;
    uint16_t interval_IMU_CALCDATA_;
    uint16_t offset_IMU_CALCDATA_;
    struct IMU_CALCDATA IMU_CALCDATA_;
    uint16_t interval_RC_DATA_;
    uint16_t offset_RC_DATA_;
    struct RC_DATA RC_DATA_;
    uint16_t interval_CONTROLLER_OUTPUT_;
    uint16_t offset_CONTROLLER_OUTPUT_;
    struct CONTROLLER_OUTPUT CONTROLLER_OUTPUT_;
    uint16_t interval_GPS_DATA_;
    uint16_t offset_GPS_DATA_;
    struct GPS_DATA GPS_DATA_;
    uint16_t interval_WAYPOINT_;
    uint16_t offset_WAYPOINT_;
    struct WAYPOINT WAYPOINT_;
    uint16_t interval_GPS_DATA_ADVANCED_;
    uint16_t offset_GPS_DATA_ADVANCED_;
    struct GPS_DATA_ADVANCED GPS_DATA_ADVANCED_;
//    uint16_t interval_CAM_DATA_ = 0;
//    uint16_t offset_CAM_DATA_ = 8;

    bool pollingEnabled_;

  };                            // end class AutoPilot
}                               //end namespace asctec_autopilot
#endif
