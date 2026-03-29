package org.neiacademy.robotics.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double leftNormalizedPosition = 0.0;
    public double leftSetpointPosition = 0.0;

    public double rightNormalizedPosition = 0.0;
    public double rightSetpointPosition = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setPositionNormalized(double normalizedPosition) {}

  public default void setSpeedNormalized(double normalizedSpeed) {}

  public default void resetPosition() {}

  public default void zeroPosition() {}

  public default void updateCurrentPosition() {}

  public default boolean isPositionWithinTolerance() {
    return true;
  }

  public default void stop() {}
}
