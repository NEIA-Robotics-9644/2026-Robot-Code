package org.neiacademy.robotics.frc2026.commands;

import java.util.function.DoubleSupplier;

import org.neiacademy.robotics.frc2026.subsystems.indexer.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IndexerCommands {

    public static Command runIndexer(Indexer indexer, DoubleSupplier normalizedVelocity, DoubleSupplier feedForward) {
        return Commands.run(
            () -> indexer.setVelocity(normalizedVelocity.getAsDouble(), feedForward.getAsDouble())
        );
    }
}
