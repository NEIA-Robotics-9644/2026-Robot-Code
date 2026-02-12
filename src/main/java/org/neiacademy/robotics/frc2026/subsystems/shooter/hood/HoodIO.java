package org.neiacademy.robotics.frc2026.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    public boolean motorConnected = true;

    public double positionRads = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setVolts(double volts) {}

  default void setVelocity(double velocity, double feedForward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setPIDSetpoint(double angleInRads) {}

  default void setOutputPIDZero() {}

  default void stop() {}
}
