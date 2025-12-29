package org.neiacademy.robotics.frc2026.subsystems.misc.GenericZeroing.HallEffect;

import org.littletonrobotics.junction.AutoLog;

public interface GenericHallEffectSensorIO {

  @AutoLog
  class HallEffectSensorIOInputs {
    public boolean triggered = false;
  }

  default boolean getValidStatus() {
    return true;
  }

  default void updateInputs(HallEffectSensorIOInputs inputs) {}
}
