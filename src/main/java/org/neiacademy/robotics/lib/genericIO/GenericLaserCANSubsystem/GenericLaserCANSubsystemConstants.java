package org.neiacademy.robotics.lib.genericIO.GenericLaserCANSubsystem;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import org.neiacademy.robotics.frc2026.util.drivers.CANDeviceID;

/** Wrapper class for TalonFX config params (Recommend initializing in a static block!) */
public class GenericLaserCANSubsystemConstants {

  public String kName = "ERROR_ASSIGN_A_NAME";

  public CANDeviceID laserCANDeviceId;

  public RangingMode rangingMode;
  public RegionOfInterest regionOfInterest;
  /*
  x and y shift the top-left corner of the ROI (not the center).
  w and h are the width and height of the ROI box.
  */
  public TimingBudget timingBudget;
}
