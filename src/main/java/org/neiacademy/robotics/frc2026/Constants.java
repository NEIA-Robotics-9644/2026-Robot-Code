// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.neiacademy.robotics.frc2026;

import edu.wpi.first.wpilibj.RobotBase;
import org.neiacademy.robotics.frc2026.util.drivers.CANDeviceID;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  // Use LoggedTunableNumbers
  public static final boolean tuningMode = true;
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final CANDeviceID kPDH_CAN_ID = new CANDeviceID(0);
  public static final CANDeviceID kRIO_CAN_ID = new CANDeviceID(1);

  public static final CANDeviceID kLASER_CAN_ID = new CANDeviceID(40);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
