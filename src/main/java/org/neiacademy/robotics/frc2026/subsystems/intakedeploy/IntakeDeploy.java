package org.neiacademy.robotics.frc2026.subsystems.intakedeploy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.Constants;
import org.neiacademy.robotics.frc2026.Presets;
import org.neiacademy.robotics.frc2026.util.Util;

public class IntakeDeploy extends SubsystemBase {

  private final IntakeDeployIO io;
  private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Alert intakeDeployMotorDisconnectedAlert =
      new Alert("IntakeDeploy motor disconnected!", Alert.AlertType.kWarning);

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeDeploy", inputs);

    intakeDeployMotorDisconnectedAlert.set(
        !motorConnectedDebouncer.calculate(inputs.motorConnected));

    if (Constants.Intake.kP.hasChanged(hashCode()) || Constants.Intake.kD.hasChanged(hashCode())) {
      io.setPID(Constants.Intake.kP.get(), Constants.Intake.kD.get());
    }
    if (Constants.Intake.kS.hasChanged(hashCode())
        || Constants.Intake.kG.hasChanged(hashCode())
        || Constants.Intake.kV.hasChanged(hashCode())
        || Constants.Intake.kA.hasChanged(hashCode())) {
      io.setFeedForward(
          Constants.Intake.kS.get(),
          Constants.Intake.kG.get(),
          Constants.Intake.kV.get(),
          Constants.Intake.kA.get());
    }
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.positionSetpointRads,
        inputs.rotorPositionRads,
        Units.degreesToRadians(Constants.Intake.POSITION_TOLERANCE.get()));
  }

  public boolean isDeployed() {
    return inputs.rotorPositionRads >= Units.degreesToRadians(45);
  }

  public Command runPositionCommand(DoubleSupplier positionRotations) {
    return run(() -> io.runPosition(positionRotations.getAsDouble())).until(this::atSetpoint);
  }

  public Command runPositionCommandWithTimeout(DoubleSupplier positionRotations) {
    return run(() -> io.runPosition(positionRotations.getAsDouble()))
        .withTimeout(Presets.Intake.SHOOTING_TOGGLE_TIMEOUT_SPEED_SEC.getAsDouble());
  }

  public Command runTrackedPositionCommand(DoubleSupplier positionRotations) {
    return run(() -> io.runPosition(positionRotations.getAsDouble()));
  }

  /** Integrates a bounded target; the motor controller holds the last target on release. */
  public Command manualPositionCommand(DoubleSupplier joystick) {
    return new Command() {
      private double target;

      {
        addRequirements(IntakeDeploy.this);
      }

      @Override
      public void initialize() {
        target = getPositionRotations();
      }

      @Override
      public void execute() {
        double min =
            Math.min(Presets.Intake.TUCK_ANGLE_DEG.get(), Presets.Intake.EXTEND_ANGLE_DEG.get());
        double max =
            Math.max(Presets.Intake.TUCK_ANGLE_DEG.get(), Presets.Intake.EXTEND_ANGLE_DEG.get());
        double duration =
            Math.max(Constants.loopTime, Presets.Intake.PIVOT_MANUAL_MOVEMENT_TOTAL_TIME.get());
        target =
            MathUtil.clamp(
                target
                    + MathUtil.applyDeadband(joystick.getAsDouble(), 0.1)
                        * (max - min)
                        / duration
                        * Constants.loopTime,
                min,
                max);
        io.runPosition(target);
      }
    };
  }

  public double getAngleRads() {
    return inputs.rotorPositionRads;
  }

  public double getPositionRotations() {
    return Units.radiansToRotations(inputs.rotorPositionRads);
  }

  public Command runVoltageCommand(DoubleSupplier volts) {
    return run(() -> io.runVoltage(volts.getAsDouble())).finallyDo(io::stop);
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
