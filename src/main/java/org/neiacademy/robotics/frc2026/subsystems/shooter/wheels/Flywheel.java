package org.neiacademy.robotics.frc2026.subsystems.shooter.wheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel {
  String name;
  private final FlywheelIO flywheel;

  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(String name, FlywheelIO flywheel) {
    this.name = name;
    this.flywheel = flywheel;
  }

  public void periodic() {
    flywheel.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public Command setAngleRads(DoubleSupplier velocity) {
    return Commands.runEnd(
        () -> flywheel.setPIDSetpoint(velocity.getAsDouble()), () -> flywheel.setOutputPIDZero());
  }

  public void setWheelsVelocity(double normalizedVelocity, double feedForward) {
    flywheel.setVelocity(normalizedVelocity, feedForward);
  }
}
