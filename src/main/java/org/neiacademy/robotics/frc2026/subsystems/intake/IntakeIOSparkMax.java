package org.neiacademy.robotics.frc2026.subsystems.intake;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax motor;

    private final RelativeEncoder encoder;

    private SparkMaxConfig config = new SparkMaxConfig();

    public IntakeIOSparkMax(int id, double reduction, int maxCurrentA) {
        this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);

        this.encoder = motor.getEncoder();

        config.idleMode(SparkMaxConfig.IdleMode.kBrake);

        config.smartCurrentLimit(maxCurrentA);

        this.motor.configure(
            config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void setVelocity(double normalizedVelocity) {
        motor.set(normalizedVelocity);
    }

    @Override
    public void periodic() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
        inputs.velocityRadsPerSec = Units.rotationsToRadians(encoder.getVelocity());
        inputs.appliedVoltage = motor.getAppliedOutput();
        inputs.outputCurrentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();
    }

    @Override
    public double getPositionRads(){
        return encoder.getPosition();
    }
}
