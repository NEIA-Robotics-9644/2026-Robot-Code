package org.neiacademy.robotics.lib.genericIO.GenericBeamBreak;

import org.littletonrobotics.junction.AutoLog;

public interface GenericBeamBreakSensorIO {

  @AutoLog
  class BeamBreakSensorIOInputs {
    public boolean triggered = false;
  }

  default boolean getValidStatus() {
    return true;
  }

  default void updateInputs(BeamBreakSensorIOInputs inputs) {}
}
