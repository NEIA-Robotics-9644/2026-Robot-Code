package org.neiacademy.robotics.frc2026.subsystems.test.HallEffect;

import org.neiacademy.robotics.lib.genericIO.GenericZeroing.HallEffect.GenericHallEffectSensorIOImpl;

public class TestHallEffectIOReal extends GenericHallEffectSensorIOImpl
    implements TestHallEffectIO {
  public TestHallEffectIOReal() {
    super(9, TriggerType.ACTIVE_LOW);
  }
}
