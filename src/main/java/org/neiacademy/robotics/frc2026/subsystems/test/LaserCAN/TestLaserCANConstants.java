package org.neiacademy.robotics.frc2026.subsystems.test.LaserCAN;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import org.neiacademy.robotics.frc2026.Constants;
import org.neiacademy.robotics.frc2026.subsystems.misc.GenericLaserCANSubsystem.GenericLaserCANSubsystemConstants;

public class TestLaserCANConstants {
  public static final GenericLaserCANSubsystemConstants kSubSysConstants =
      new GenericLaserCANSubsystemConstants();

  static {
    kSubSysConstants.kName = "TestLaserCAN";
    kSubSysConstants.laserCANDeviceId = Constants.kLASER_CAN_ID;
    kSubSysConstants.rangingMode = RangingMode.SHORT;
    kSubSysConstants.regionOfInterest = new RegionOfInterest(8, 8, 16, 4);
    /*
    x and y shift the top-left corner of the ROI (not the center).
    w and h are the width and height of the ROI box.
    */
    kSubSysConstants.timingBudget = TimingBudget.TIMING_BUDGET_20MS;
  }
}
