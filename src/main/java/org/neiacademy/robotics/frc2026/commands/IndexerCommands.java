package org.neiacademy.robotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.neiacademy.robotics.frc2026.subsystems.indexer.Indexer;

public class IndexerCommands {

  public static Command runIndexer(
      Indexer indexer, DoubleSupplier normalizedVelocity, DoubleSupplier feedForward) {

    return Commands.runEnd(
        () -> indexer.setVelocity(normalizedVelocity.getAsDouble(), feedForward.getAsDouble()),
        () -> indexer.setVelocity(0, 0),
        indexer);
  }
}
