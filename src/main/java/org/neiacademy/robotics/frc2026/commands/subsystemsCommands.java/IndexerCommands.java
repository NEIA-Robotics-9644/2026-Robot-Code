
package org.neiacademy.robotics.frc2026.commands.IndexerCommands.java;

import edu.wpi.first.wpilibj2.command.Command;
import org.neiacademy.robotics.frc2026.subsystems.intake.Intake;


public class IndexerCommands extends Command {

  private final Intake intake;
  private final double velocity;

  
  public IndexerCommands(Indexer Indexer, double velocity) {
    this.Indexer = Indexer;
    this.velocity = velocity;

    addRequirements(Indexer);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    
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