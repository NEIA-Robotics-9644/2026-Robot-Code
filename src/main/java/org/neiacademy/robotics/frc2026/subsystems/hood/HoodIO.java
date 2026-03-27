package org.neiacademy.robotics.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double leftNormalizedSpeed = 0.0;
    public double leftNormalizedPosition = 0.0;
    public double leftPositionMillimeters = 0.0;

    public double rightNormalizedSpeed = 0.0;
    public double rightNormalizedPosition = 0.0;
    public double rightPositionMillimeters = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setSpeed(double normalizedSpeed) {}

  public default void setPositionNormalized(double normalizedPosition) {}

  public default void setPosition(double position) {}

  public default void setPositionMillimeters(double setpoint) {}

  public default void setDisabled() {}

  public default void setBounds(int max, int deadbandMax, int center, int deadbandMin, int min) {}

  public default void resetPosition() {}

  public default void stop() {}
}
