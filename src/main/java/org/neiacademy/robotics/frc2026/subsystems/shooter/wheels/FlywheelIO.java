package org.neiacademy.robotics.frc2026.subsystems.shooter.wheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public boolean motorConnected = true;

    public double positionRads = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setVolts(double volts) {}

  default void setVelocity(double velocity, double feedForward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setPIDSetpoint(double velocity) {}

  default void setOutputPIDZero() {}

  default void followFlywheel(FlywheelIO leader, boolean ignoreInvert) {}

  default void stop() {}
}
