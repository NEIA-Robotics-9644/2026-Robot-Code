package org.neiacademy.robotics.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double normalizedSpeed = 0.0;
    public double angleDegrees = 0.0;
    public double normalizedPosition = 0.0;
    public double rawPWM = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setSpeed(double normalizedSpeed) {}

  public default void setPositionNormalized(double normalizedPosition) {}
  
  public default void setPositionMillimeters(double setpoint) {}

  public default void setDisabled() {}

  public default void setBounds(int max, int deadbandMax, int center, int deadbandMin, int min) {}

  public default void setAngle(double angleDegrees) {}

  public default void stop() {}
}
