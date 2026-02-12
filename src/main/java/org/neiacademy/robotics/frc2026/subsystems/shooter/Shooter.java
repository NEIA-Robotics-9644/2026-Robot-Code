package org.neiacademy.robotics.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.neiacademy.robotics.frc2026.subsystems.shooter.hood.Hood;
import org.neiacademy.robotics.frc2026.subsystems.shooter.wheels.Flywheel;

public class Shooter extends SubsystemBase {
  @Getter private final Hood leftHood;
  @Getter private final Hood rightHood;
  @Getter private final Flywheel leftFlywheel;
  @Getter private final Flywheel rightFlywheel;

  public Shooter(Hood leftHood, Hood rightHood, Flywheel leftFlywheel, Flywheel rightFlywheel) {
    this.leftHood = leftHood;
    this.rightHood = rightHood;
    this.leftFlywheel = leftFlywheel;
    this.rightFlywheel = rightFlywheel;
  }

  public void periodic() {}

  public void setHoodVelocity(double normalizedVelocity, double feedForward, HoodSide side) {
    if (side == HoodSide.LEFT_HOOD) {
      leftHood.setVelocity(normalizedVelocity, feedForward);
    } else {
      rightHood.setVelocity(normalizedVelocity, feedForward);
    }
  }

  public void setFlywheelVelocity(double normalizedVelocity, double feedForward, FlywheelSide side) {
    if (side == FlywheelSide.LEFT_FLYWHEEL) {
      leftHood.setVelocity(normalizedVelocity, feedForward);
    } else {
      rightHood.setVelocity(normalizedVelocity, feedForward);
    }
  }


  public enum HoodSide {
    LEFT_HOOD,
    RIGHT_HOOD
  };

  public enum FlywheelSide {
    LEFT_FLYWHEEL,
    RIGHT_FLYWHEEL
  };
}
