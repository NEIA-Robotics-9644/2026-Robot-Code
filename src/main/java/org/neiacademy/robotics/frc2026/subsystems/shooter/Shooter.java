package org.neiacademy.robotics.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.neiacademy.robotics.frc2026.subsystems.shooter.wheels.Flywheel;
import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  // private final Hood leftHood;
  // private final Hood rightHood;

  private final Flywheel leftFlywheel;
  private final Flywheel rightFlywheel;
  private final Flywheel leftFollower;
  private final Flywheel rightFollower;

  private final Flywheel leftRollers;
  private final Flywheel rightRollers;

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
      Flywheel leftRollers,
      Flywheel rightRollers) {
    // this.leftHood = leftHood;
    // this.rightHood = rightHood;

    this.leftFlywheel = leftFlywheel;
    this.rightFlywheel = rightFlywheel;
    this.leftFollower = leftFollower;
    this.rightFollower = rightFollower;

    this.leftRollers = leftRollers;
    this.rightRollers = rightRollers;

    leftFollower.followFlywheel(leftFlywheel, false);
    rightFollower.followFlywheel(rightFlywheel, false);
  }

  public void periodic() {}

  /*public void setHoodVelocity(
      double normalizedVelocity, double feedForward, HoodSide side) {
      switch (side) {
        case LEFT_HOOD:
            leftHood.setVelocity(normalizedVelocity, feedForward);
        case RIGHT_HOOD:
            rightHood.setVelocity(normalizedVelocity, feedForward);
        default:
            throw new IllegalStateException(side + " is not an option for setHoodVelocity.");
      }
  }*/

  /*public Command setHoodAngleSetpoint(
      DoubleSupplier angleDegrees, HoodSide side) {
      switch (side) {
        case LEFT_HOOD:
            return leftHood.setAngleDegrees(angleDegrees);
        case RIGHT_HOOD:
            return rightHood.setAngleDegrees(angleDegrees);
        default:
            throw new IllegalStateException(side + " is not an option for setHoodAngleSetpoint.");
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
      case LEFT_ROLLERS:
        leftRollers.setVelocity(normalizedVelocity, feedForward);
      case RIGHT_ROLLERS:
        rightRollers.setVelocity(normalizedVelocity, feedForward);
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
      case LEFT_ROLLERS:
        return leftRollers.setSpeedSetpoint(normalizedVelocity);
      case RIGHT_ROLLERS:
        return rightRollers.setSpeedSetpoint(normalizedVelocity);
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
      case LEFT_ROLLERS:
        leftRollers.setOutputPIDZero();
      case RIGHT_ROLLERS:
        rightRollers.setOutputPIDZero();
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
      case LEFT_ROLLERS:
        leftRollers.getVelocityPercentToGoal();
      case RIGHT_ROLLERS:
        rightRollers.getVelocityPercentToGoal();
      default:
        throw new IllegalStateException(side + " is not an option for getFlywheelVelocityPercent.");
    }
  }

  public enum HoodSide {
    LEFT_HOOD,
    RIGHT_HOOD
  };

  public enum FlywheelSide {
    LEFT_FLYWHEEL,
    RIGHT_FLYWHEEL,
    LEFT_FOLLOWER,
    RIGHT_FOLLOWER,
    LEFT_ROLLERS,
    RIGHT_ROLLERS
  };
}
