package org.neiacademy.robotics.frc2026.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.Constants;
import org.neiacademy.robotics.frc2026.util.Util;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final boolean isLeftShooter;
  private boolean idleActive;
  private boolean idleCoasting;
  private double idleTargetRadsPerSec;
  private double supplyEnergyJoules;
  private double shootingTargetRadsPerSec = Double.NaN;

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert shooterLeaderDisconnectedAlert;
  private final Alert shooterFollowerDisconnectedAlert;

  public Shooter(ShooterIO io, boolean isLeftShooter) {
    this.io = io;
    this.isLeftShooter = isLeftShooter;

    shooterLeaderDisconnectedAlert =
        new Alert(
            (isLeftShooter ? "Left" : "Right") + "ShooterLeader motor disconnected!",
            Alert.AlertType.kWarning);
    shooterFollowerDisconnectedAlert =
        new Alert(
            (isLeftShooter ? "Left" : "Right") + "ShooterFollower motor disconnected!",
            Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/" + (isLeftShooter ? "Left" : "Right"), inputs);
    String logKey = "Shooter/" + (isLeftShooter ? "Left" : "Right");
    double supplyPowerWatts =
        inputs.supplyVoltageVolts
            * (inputs.leaderSupplyCurrentAmps + inputs.followerSupplyCurrentAmps);
    supplyEnergyJoules += supplyPowerWatts * Constants.loopTime;
    Logger.recordOutput(logKey + "/SupplyPowerWatts", supplyPowerWatts);
    Logger.recordOutput(logKey + "/SupplyEnergyJoules", supplyEnergyJoules);
    Logger.recordOutput(logKey + "/IdleActive", idleActive);
    Logger.recordOutput(logKey + "/IdleCoasting", idleCoasting);
    Logger.recordOutput(
        logKey + "/IdleTargetRPM",
        Units.radiansPerSecondToRotationsPerMinute(idleTargetRadsPerSec));

    shooterLeaderDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.leaderConnected));
    shooterFollowerDisconnectedAlert.set(
        !motorConnectedDebouncer.calculate(inputs.followerConnected));

    if (isLeftShooter) {
      if (Constants.Shooter.LEFT_kP.hasChanged(hashCode())
          || Constants.Shooter.LEFT_kD.hasChanged(hashCode())) {
        io.setPID(Constants.Shooter.LEFT_kP.get(), Constants.Shooter.LEFT_kD.get());
      }
      if (Constants.Shooter.LEFT_kS.hasChanged(hashCode())
          || Constants.Shooter.LEFT_kV.hasChanged(hashCode())
          || Constants.Shooter.LEFT_kA.hasChanged(hashCode())) {
        io.setFeedForward(
            Constants.Shooter.LEFT_kS.get(),
            0.0,
            Constants.Shooter.LEFT_kV.get(),
            Constants.Shooter.LEFT_kA.get());
      }
    } else {
      if (Constants.Shooter.RIGHT_kP.hasChanged(hashCode())
          || Constants.Shooter.RIGHT_kD.hasChanged(hashCode())) {
        io.setPID(Constants.Shooter.RIGHT_kP.get(), Constants.Shooter.RIGHT_kD.get());
      }
      if (Constants.Shooter.RIGHT_kS.hasChanged(hashCode())
          || Constants.Shooter.RIGHT_kV.hasChanged(hashCode())
          || Constants.Shooter.RIGHT_kA.hasChanged(hashCode())) {
        io.setFeedForward(
            Constants.Shooter.RIGHT_kS.get(),
            0.0,
            Constants.Shooter.RIGHT_kV.get(),
            Constants.Shooter.RIGHT_kA.get());
      }
    }
  }

  public boolean atSetpoint() {
    return Double.isFinite(shootingTargetRadsPerSec)
        && shootingTargetRadsPerSec > 0.0
        && Util.epsilonEquals(
            shootingTargetRadsPerSec,
            inputs.leaderVelocityRadsPerSec,
            Constants.Shooter.VELOCITY_TOLERANCE.get());
  }

  public Command runVelocityCommand(DoubleSupplier velocityRadsPerSec) {
    return runTrackedVelocityCommand(velocityRadsPerSec).until(this::atSetpoint);
  }

  public Command runTrackedVelocityCommand(DoubleSupplier velocityRadsPerSec) {
    return run(() -> {
          shootingTargetRadsPerSec = velocityRadsPerSec.getAsDouble();
          io.runVelocity(shootingTargetRadsPerSec);
        })
        .beforeStarting(() -> shootingTargetRadsPerSec = velocityRadsPerSec.getAsDouble())
        .finallyDo(() -> shootingTargetRadsPerSec = Double.NaN);
  }

  /** Coast down to a low-speed floor, then maintain it until a shooting command takes over. */
  public Command idleVelocityCommand(BooleanSupplier enabled, DoubleSupplier velocityRadsPerSec) {
    return run(() -> {
          double target = velocityRadsPerSec.getAsDouble();
          idleTargetRadsPerSec = Double.isFinite(target) ? Math.max(0.0, target) : 0.0;
          idleActive = enabled.getAsBoolean() && idleTargetRadsPerSec > 0.0;
          idleCoasting = idleActive && inputs.leaderVelocityRadsPerSec > idleTargetRadsPerSec;
          if (!idleActive || idleCoasting) {
            stop();
          } else {
            io.runIdleVelocity(idleTargetRadsPerSec);
          }
        })
        .finallyDo(
            () -> {
              idleActive = false;
              idleCoasting = false;
              stop();
            });
  }

  public void stop() {
    shootingTargetRadsPerSec = Double.NaN;
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
