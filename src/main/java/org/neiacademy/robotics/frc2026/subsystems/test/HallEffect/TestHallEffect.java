package org.neiacademy.robotics.frc2026.subsystems.test.HallEffect;

import org.neiacademy.robotics.lib.genericIO.GenericZeroing.HallEffect.GenericHallEffectSensor;

public class TestHallEffect extends GenericHallEffectSensor {

  public TestHallEffect(TestHallEffectIO io) {
    super("testhall", io);
  }

  /** Convenience method for test commands / dashboards */
  public boolean isHallTriggered() {
    return isTriggered();
  }
}
