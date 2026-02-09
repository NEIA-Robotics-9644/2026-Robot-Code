package org.neiacademy.robotics.frc2026.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;
public interface IndexerIO{
    @AutoLog
    static class IndexerIOInputs {
        public double velocityRadsPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double outputCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }
    default void updateInputs(IndexerIOInputs inputs) {}

    public default void setVelocity(double normalizedVelocity) {}

    public default double getVelocityPercent() {
        return 0.0;
    }

    public default void periodic() {}

    public default void setBrakeMode(boolean brake) {}

}
