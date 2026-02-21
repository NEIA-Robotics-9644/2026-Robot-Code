package org.neiacademy.robotics.frc2026.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public boolean motorConnected = true;

    public double positionRads = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setVolts(double volts) {}

  default void setVelocity(double velocity, double feedForward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setPIDSetpoint(double angleInRads) {}

  default void setOutputPIDZero() {}

  default void stop() {}
}
