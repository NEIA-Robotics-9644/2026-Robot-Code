package org.neiacademy.robotics.frc2026.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import org.neiacademy.robotics.frc2026.subsystems.Superstructure;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;

/** Mechanism and clock events for the native PathPlanner bump .auto files. */
public final class NerdBumpAutos {
  private NerdBumpAutos() {}

  public static void registerCommands(Drive drive, Superstructure superstructure) {
    // Path marker starts mechanism-only preparation once the robot lands and begins turning.
    // PathPlanner keeps ownership of chassis rotation and cancels this at path completion.
    NamedCommands.registerCommand("Bump Spin up", superstructure.hubSpinFlywheelsCommand());
    // Only one autonomous routine runs at a time. Restart on every scheduling, including reruns.
    Timer elapsed = new Timer();
    NamedCommands.registerCommand("Bump Start clock", Commands.runOnce(elapsed::restart));
    for (int seconds : new int[] {17, 20}) {
      NamedCommands.registerCommand(
          "Bump Until " + seconds + " seconds",
          Commands.waitUntil(() -> elapsed.hasElapsed(seconds)));
    }
    // This event has no requirements: its enclosing auto already owns drive and mechanisms.
    NamedCommands.registerCommand(
        "Bump Cleanup",
        Commands.startEnd(
            () -> {},
            () -> {
              elapsed.stop();
              drive.stop();
              superstructure.stopAutoFuelHandling();
            }));
  }
}
