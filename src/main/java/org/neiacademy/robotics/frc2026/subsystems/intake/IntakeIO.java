package org.neiacademy.robotics.frc2026.subsystems.intake;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    static class IntakeIOInputs {
        public double positionRads = 0.0;
        public double velocityRadsPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double outputCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }
    default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVelocity(double normalizedVelocity) {}

    public default double getVelocityPercent() {
        return 0.0;
    }

    public default void periodic() {}

    public default void setBrakeMode(boolean brake) {}

    public default double getPositionRads(){
        return 0.0;
    }
}
