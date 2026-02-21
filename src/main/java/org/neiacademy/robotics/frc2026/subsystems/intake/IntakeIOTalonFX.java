package org.neiacademy.robotics.frc2026.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX motor;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  private final boolean useCancoder;

  private double pidSetpointRads;

  public IntakeIOTalonFX(int id, CANBus canBus, boolean inverted) {
    motor = new TalonFX(id, canBus);
    cancoder = null;
    this.useCancoder = false;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(config, 1.0);
    motor.getConfigurator().apply(controllerConfig, 1.0);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    tempCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent, tempCelsius);
  }

  public IntakeIOTalonFX(
      int id, int canId, CANBus canBus, boolean invertedMotor, boolean invertedCancoder) {
    motor = new TalonFX(id, canBus);
    cancoder = new CANcoder(canId);
    this.useCancoder = true;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        invertedMotor ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();

    config.Feedback.SensorToMechanismRatio = 1.0;

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    cancoderConfig.MagnetSensor.SensorDirection =
        invertedCancoder
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    cancoder.getConfigurator().apply(cancoderConfig);

    motor.getConfigurator().apply(config, 1.0);
    motor.getConfigurator().apply(controllerConfig, 1.0);

    position = cancoder.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    tempCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent, tempCelsius);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    if (useCancoder) {
      inputs.motorConnected =
          BaseStatusSignal.refreshAll(
                  velocity, appliedVolts, supplyCurrent, torqueCurrent, tempCelsius)
              .isOK();
      inputs.cancoderConnected = BaseStatusSignal.refreshAll(position).isOK();
    } else {
      inputs.motorConnected =
          BaseStatusSignal.refreshAll(
                  position, velocity, appliedVolts, supplyCurrent, torqueCurrent, tempCelsius)
              .isOK();
    }

    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRpm = velocity.getValueAsDouble() * 60.0;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void setVolts(double volts) {
    motor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocity, double feedForward) {
    motor.setControl(velocityControl.withVelocity(velocity).withFeedForward(feedForward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.kP = kP;
    controllerConfig.kI = kI;
    controllerConfig.kD = kD;
    motor.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void setPIDSetpoint(double angleInRads) {
    motor.setControl(new PositionVoltage(angleInRads).withSlot(0).withEnableFOC(true));
    pidSetpointRads = angleInRads;
  }

  @Override
  public void setOutputPIDZero() {
    motor.setControl(new DutyCycleOut(0));
  }

  @Override
  public double getPIDSetpoint() {
    return pidSetpointRads;
  }

  @Override
  public void stop() {
    motor.setControl(neutralControl);
  }
}
