package org.neiacademy.robotics.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean motorConnected = true;
    public boolean cancoderConnected = false;

    public double positionRads = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setVolts(double volts) {}

  default void setVelocity(double velocity, double feedForward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setPIDSetpoint(double angleInRads) {}

  default double getPIDSetpoint() {
    return 0.0;
  }

  default void setOutputPIDZero() {}

  default void stop() {}
}
