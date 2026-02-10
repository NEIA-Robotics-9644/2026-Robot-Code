package org.neiacademy.robotics.frc2026.commands.SubsystemCommands.java;

import edu.wpi.first.wpilibj2.command.Command;
import org.neiacademy.robotics.frc2026.subsystems.intake.Intake;


public class IntakeCommands extends Command {

  private final Intake intake;
  private final double velocity;

  
  public IntakeCommands(Intake intake, double velocity) {
    this.intake = intake;
    this.velocity = velocity;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    intake.setWheelVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setWheelVelocity(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}