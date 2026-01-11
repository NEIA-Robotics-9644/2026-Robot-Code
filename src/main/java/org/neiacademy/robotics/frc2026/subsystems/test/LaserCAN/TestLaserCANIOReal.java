package org.neiacademy.robotics.frc2026.subsystems.test.LaserCAN;

import org.neiacademy.robotics.lib.genericIO.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class TestLaserCANIOReal extends GenericLaserCANSubsystemIOImpl implements TestLaserCANIO {

  public TestLaserCANIOReal() {
    super(TestLaserCANConstants.kSubSysConstants, false);
  }
}
