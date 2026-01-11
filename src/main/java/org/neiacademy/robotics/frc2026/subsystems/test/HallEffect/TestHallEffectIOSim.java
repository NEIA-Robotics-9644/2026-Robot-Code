package org.neiacademy.robotics.frc2026.subsystems.test.HallEffect;

import org.neiacademy.robotics.lib.genericIO.GenericZeroing.HallEffect.GenericHallEffectSensorIOImpl;

public class TestHallEffectIOSim extends GenericHallEffectSensorIOImpl implements TestHallEffectIO {

  public TestHallEffectIOSim() {
    super(TestHallEffectConstants.kSubSysConstants, true);
  }
}
