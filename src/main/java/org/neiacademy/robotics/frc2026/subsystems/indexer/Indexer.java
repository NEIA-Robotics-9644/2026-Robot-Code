package org.neiacademy.robotics.frc2026.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO wheels;

  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO wheels) {
    this.wheels = wheels;
  }

  public void periodic() {
    wheels.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public void setVelocity(double velocity, double feedForward) {
    wheels.setVelocity(velocity, feedForward);
  }
}
