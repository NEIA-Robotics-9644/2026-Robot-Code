package org.neiacademy.robotics.frc2026.subsystems.test.HallEffect;

import org.neiacademy.robotics.frc2026.subsystems.misc.GenericZeroing.HallEffect.GenericHallEffectSensorSubsystem;

public class TestHallEffect extends GenericHallEffectSensorSubsystem {

  public TestHallEffect(TestHallEffectIO io) {
    super("testhall", io);
  }

  /** Convenience method for test commands / dashboards */
  public boolean isHallTriggered() {
    return isTriggered();
  }
}
