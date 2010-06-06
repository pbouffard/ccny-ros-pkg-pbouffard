/*
 *  AscTec Autopilot IMU Calibration
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

#ifndef IMU_CALIBRATE_H
#define IMU_CALIBRATE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>

#include "asctec_autopilot/serial_interface.h"
#include "asctec_autopilot/asctec_autopilot.h"

class statData
{
public:

  int iterations;

  double ave;
  double sum;
  double sumsq;
  double stdev;
  double *val;

  const char *name;
  const char *units;

    statData (int i, const char *name, const char *units)
  {
    this->name = name;
    this->units = units;

    iterations = i;

    val = new double[iterations];
  }

  void update ()
  {
    updateSum ();
    updateAve ();
    updateSumsq ();
    updateStdev ();
  }

  void updateSum ()
  {
    sum = 0.0;

    for (int i = 0; i < iterations; i++)
      sum += val[i];
  }

  void updateAve ()
  {
    ave = sum / (double) iterations;
  }

  void updateSumsq ()
  {
    sumsq = 0.0;

    for (int i = 0; i < iterations; i++)
      sumsq += (ave - val[i]) * (ave - val[i]);
  }

  void updateStdev ()
  {
    stdev = sqrt (sumsq / (double) iterations);
  }

  void print ()
  {
    using std::cout;
    using std::setw;
    using std::endl;
    cout << setw (15) << name << "\t";
    cout << setw (8) << units << "\t";
    cout << setw (15) << ave << "\t";
    cout << setw (15) << stdev << endl;
  }

  static void printHeader ()
  {
    using std::cout;
    using std::setw;
    using std::endl;
    cout << "-------------------------------------------------------------------" << endl;
    cout << setw (15) << "NAME" << "\t";
    cout << setw (8) << "UNITS" << "\t";
    cout << setw (15) << "AVERAGE" << "\t";
    cout << setw (15) << "STD DEV" << endl;
    cout << "-------------------------------------------------------------------" << endl;
  }
};

#endif
