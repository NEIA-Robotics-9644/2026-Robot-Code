package org.neiacademy.robotics.frc2026.subsystems.shooter.wheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;

public class Flywheel {
  @Getter String name;
  @Getter private final FlywheelIO flywheel;

  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private double currentVelocityGoal = 0;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  public Flywheel(String name, FlywheelIO flywheel) {
    this.name = name;
    this.flywheel = flywheel;

    kP = new LoggedTunableNumber(name + "/kP");
    kI = new LoggedTunableNumber(name + "/kI");
    kD = new LoggedTunableNumber(name + "/kD");

    flywheel.setPID(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
  }

  public void periodic() {
    flywheel.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public Command setSpeedSetpoint(DoubleSupplier velocity) {
    return Commands.runEnd(
        () -> {
          flywheel.setPIDSetpoint(velocity.getAsDouble());
          currentVelocityGoal = velocity.getAsDouble();
        },
        () -> {
          flywheel.setOutputPIDZero();
          currentVelocityGoal = 0;
        });
  }

  public void setOutputPIDZero() {
    flywheel.setOutputPIDZero();
  }

  public void setVelocity(double normalizedVelocity, double feedForward) {
    flywheel.setVelocity(normalizedVelocity, feedForward);
  }

  public void followFlywheel(Flywheel leader, boolean ignoreInvert) {
    flywheel.followFlywheel(leader.getFlywheel(), false);
  }

  public double getVelocityPercentToGoal() {
    return Math.abs(currentVelocityGoal) / inputs.velocityRpm / 60.0;
  }
}
