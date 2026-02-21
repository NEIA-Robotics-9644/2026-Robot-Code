package org.neiacademy.robotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.neiacademy.robotics.frc2026.subsystems.intake.Intake;

public class IntakeCommands {

  public static Command runIntake(
      Intake intake, DoubleSupplier normalizedVelocity, DoubleSupplier feedForward) {
    return Commands.run(
        () ->
            intake.setWheelsVelocity(normalizedVelocity.getAsDouble(), feedForward.getAsDouble()));
  }
}
