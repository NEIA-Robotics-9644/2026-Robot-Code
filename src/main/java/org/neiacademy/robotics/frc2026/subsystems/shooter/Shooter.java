package org.neiacademy.robotics.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.neiacademy.robotics.frc2026.subsystems.shooter.wheels.Flywheel;
import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  // private final Hood hood;

  private final Flywheel leftFlywheel;
  private final Flywheel rightFlywheel;
  private final Flywheel leftFollower;
  private final Flywheel rightFollower;

  private final Flywheel feeder;

  @Getter
  private static final LoggedTunableNumber flywheelSpeedPercent =
      new LoggedTunableNumber("Shooter/FlywheelSpeedPercent");

  public Shooter(
      /*Hood leftHood,
      Hood rightHood,*/
      Flywheel leftFlywheel,
      Flywheel rightFlywheel,
      Flywheel leftFollower,
      Flywheel rightFollower,
      Flywheel feeder) {
    // this.leftHood = leftHood;
    // this.rightHood = rightHood;

    this.leftFlywheel = leftFlywheel;
    this.rightFlywheel = rightFlywheel;
    this.leftFollower = leftFollower;
    this.rightFollower = rightFollower;

    this.feeder = feeder;

    leftFollower.followFlywheel(leftFlywheel, false);
    rightFollower.followFlywheel(rightFlywheel, false);
  }

  public void periodic() {}

  /*public void setHoodVelocity(
      double normalizedVelocity, double feedForward) {
        hood.setVelocity(normalizedVelocity, feedForward);
      }
  }*/

  /*public Command setHoodAngleSetpoint(
      DoubleSupplier angleDegrees) {
        return hood.setAngleDegrees(angleDegrees);
    }
  }*/

  public void setFlywheelVelocity(
      double normalizedVelocity, double feedForward, FlywheelSide side) {
    switch (side) {
      case LEFT_FLYWHEEL:
        leftFlywheel.setVelocity(normalizedVelocity, feedForward);
      case RIGHT_FLYWHEEL:
        rightFlywheel.setVelocity(normalizedVelocity, feedForward);
      case LEFT_FOLLOWER:
        leftFollower.setVelocity(normalizedVelocity, feedForward);
      case RIGHT_FOLLOWER:
        rightFollower.setVelocity(normalizedVelocity, feedForward);
      case FEEDER:
        feeder.setVelocity(normalizedVelocity, feedForward);
      default:
        throw new IllegalStateException(side + " is not an option for setFlywheelVelocity.");
    }
  }

  public Command setFlywheelSpeedSetpoint(DoubleSupplier normalizedVelocity, FlywheelSide side) {
    switch (side) {
      case LEFT_FLYWHEEL:
        return leftFlywheel.setSpeedSetpoint(normalizedVelocity);
      case RIGHT_FLYWHEEL:
        return rightFlywheel.setSpeedSetpoint(normalizedVelocity);
      case FEEDER:
        return feeder.setSpeedSetpoint(normalizedVelocity);
      default:
        throw new IllegalStateException(side + " is not an option for setFlywheelSpeedSetpoint.");
    }
  }

  public void setFlywheelOutputPIDZero(FlywheelSide side) {
    switch (side) {
      case LEFT_FLYWHEEL:
        leftFlywheel.setOutputPIDZero();
      case RIGHT_FLYWHEEL:
        rightFlywheel.setOutputPIDZero();
      case FEEDER:
        feeder.setOutputPIDZero();
      default:
        throw new IllegalStateException(side + " is not an option for setFlywheelOutputPIDZero.");
    }
  }

  public double getFlywheelVelocityPercent(FlywheelSide side) {
    switch (side) {
      case LEFT_FLYWHEEL:
        leftFlywheel.getVelocityPercentToGoal();
      case RIGHT_FLYWHEEL:
        rightFlywheel.getVelocityPercentToGoal();
      case FEEDER:
        feeder.getVelocityPercentToGoal();
      default:
        throw new IllegalStateException(side + " is not an option for getFlywheelVelocityPercent.");
    }
  }

  public enum FlywheelSide {
    LEFT_FLYWHEEL,
    RIGHT_FLYWHEEL,
    LEFT_FOLLOWER,
    RIGHT_FOLLOWER,
    FEEDER
  };
}
