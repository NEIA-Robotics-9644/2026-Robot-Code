package org.neiacademy.robotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.neiacademy.robotics.frc2026.subsystems.shooter.Shooter;
import org.neiacademy.robotics.frc2026.subsystems.shooter.Shooter.FlywheelSide;

public class ShootWhenAtSpeedPercent extends Command {
  private final Shooter shooter;
  private final DoubleSupplier feederNormalizedVelocity;
  private final DoubleSupplier feederFeedForward;
  private final DoubleSupplier percentOffToOnThreshold;
  private final DoubleSupplier percentOnToOffThreshold;

  private boolean leftFlywheelRunning = false;
  private boolean rightFlywheelRunning = false;

  public ShootWhenAtSpeedPercent(
      Shooter shooter,
      DoubleSupplier feederNormalizedVelocity,
      DoubleSupplier feederFeedForward,
      DoubleSupplier percentOffToOnThreshold,
      DoubleSupplier percentOnToOffThreshold) {

    this.shooter = shooter;
    this.feederNormalizedVelocity = feederNormalizedVelocity;
    this.feederFeedForward = feederFeedForward;
    this.percentOffToOnThreshold = percentOffToOnThreshold;
    this.percentOnToOffThreshold = percentOnToOffThreshold;
  }

  @Override
  public void initialize() {
    // System.out.println("Shoot When Ready Cmd Initialized");
  }

  @Override
  public void execute() {
    double leftFlywheelPerecentage = shooter.getFlywheelVelocityPercent(FlywheelSide.LEFT_FLYWHEEL);

    double rightFlywheelPerecentage = shooter.getFlywheelVelocityPercent(FlywheelSide.RIGHT_FLYWHEEL);

    if (leftFlywheelPerecentage > percentOffToOnThreshold.getAsDouble() && !leftFlywheelRunning) {
      leftFlywheelRunning = true;
    } else if (leftFlywheelPerecentage < percentOnToOffThreshold.getAsDouble()
        && leftFlywheelRunning) {
      leftFlywheelRunning = false;
    }
    if (rightFlywheelPerecentage > percentOffToOnThreshold.getAsDouble() && !rightFlywheelRunning) {
      rightFlywheelRunning = true;
    } else if (rightFlywheelPerecentage < percentOnToOffThreshold.getAsDouble()
        && rightFlywheelRunning) {
      rightFlywheelRunning = false;
    }
    /*
        Make sure not to override other commands that might be using the feeder wheel at the same time
        This is why we don't stop the feeder wheel even if the shooter wheels are off
    */
    if (leftFlywheelRunning && rightFlywheelRunning) {
      shooter.setFlywheelVelocity(
          feederNormalizedVelocity.getAsDouble(),
          feederFeedForward.getAsDouble(),
          FlywheelSide.FEEDER);
    } else {
      shooter.setFlywheelVelocity(0, 0, FlywheelSide.FEEDER);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println("ShootWhenAtSpeedPercent " + (intersrupted ? "Interrupted" : "Ended"));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
