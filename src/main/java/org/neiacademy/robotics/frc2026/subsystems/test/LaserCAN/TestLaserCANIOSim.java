package org.neiacademy.robotics.frc2026.subsystems.test.LaserCAN;

import org.neiacademy.robotics.lib.genericIO.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class TestLaserCANIOSim extends GenericLaserCANSubsystemIOImpl implements TestLaserCANIO {

  public TestLaserCANIOSim() {
    super(TestLaserCANConstants.kSubSysConstants, true);
  }
}
