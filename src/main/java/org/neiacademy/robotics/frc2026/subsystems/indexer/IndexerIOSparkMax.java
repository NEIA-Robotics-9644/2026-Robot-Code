package org.neiacademy.robotics.frc2026.subsystems.indexer;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;


public class IndexerIOSparkMax implements IndexerIO{
    private final SparkMax motor;


    private SparkMaxConfig config = new SparkMaxConfig();

    public IndexerIOSparkMax(int id, double reduction, int maxCurrentA) {
        this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);


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
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput();
        inputs.outputCurrentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();
    }

}
