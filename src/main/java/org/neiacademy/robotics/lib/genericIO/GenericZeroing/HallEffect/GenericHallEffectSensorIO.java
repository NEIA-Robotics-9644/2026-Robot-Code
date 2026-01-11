package org.neiacademy.robotics.lib.genericIO.GenericZeroing.HallEffect;

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
