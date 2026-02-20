package org.neiacademy.robotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.neiacademy.robotics.frc2026.subsystems.shooter.Shooter;
import org.neiacademy.robotics.frc2026.subsystems.shooter.Shooter.FlywheelSide;

public class ShootWhenAtSpeedPercent extends Command {
  private final Shooter shooter;
  private final DoubleSupplier rollerNormalizedVelocity;
  private final DoubleSupplier rollerFeedForward;
  private final DoubleSupplier percentOffToOnThreshold;
  private final DoubleSupplier percentOnToOffThreshold;

  private boolean leftFlywheelRunning = false;
  private boolean rightFlywheelRunning = false;

  public ShootWhenAtSpeedPercent(
      Shooter shooter,
      DoubleSupplier rollerNormalizedVelocity,
      DoubleSupplier rollerFeedForward,
      DoubleSupplier percentOffToOnThreshold,
      DoubleSupplier percentOnToOffThreshold) {

    this.shooter = shooter;
    this.rollerNormalizedVelocity = rollerNormalizedVelocity;
    this.rollerFeedForward = rollerFeedForward;
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
    double rightFlywheelPerecentage =
        shooter.getFlywheelVelocityPercent(FlywheelSide.RIGHT_FLYWHEEL);

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
    if (leftFlywheelRunning) {
      shooter.setFlywheelVelocity(
          rollerNormalizedVelocity.getAsDouble(),
          rollerFeedForward.getAsDouble(),
          FlywheelSide.LEFT_ROLLERS);
    } else {
      shooter.setFlywheelVelocity(0, 0, FlywheelSide.LEFT_ROLLERS);
    }

    if (rightFlywheelRunning) {
      shooter.setFlywheelVelocity(
          rollerNormalizedVelocity.getAsDouble(),
          rollerFeedForward.getAsDouble(),
          FlywheelSide.RIGHT_ROLLERS);
    } else {
      shooter.setFlywheelVelocity(0, 0, FlywheelSide.RIGHT_ROLLERS);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println("ShootWhenAtSpeedPercent " + (interrupted ? "Interrupted" : "Ended"));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
