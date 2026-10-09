package org.neiacademy.robotics.frc2026.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/** Defers a bumper tap until release, remembering any manual-pivot use during the press. */
public class ManualPivotBumperCommand extends Command {
  private final BooleanSupplier bumperPressed;
  private final BooleanSupplier manualRequested;
  private final Runnable onTap;
  private boolean usedForManual;

  public ManualPivotBumperCommand(
      BooleanSupplier bumperPressed, BooleanSupplier manualRequested, Runnable onTap) {
    this.bumperPressed = bumperPressed;
    this.manualRequested = manualRequested;
    this.onTap = onTap;
  }

  @Override
  public void initialize() {
    usedForManual = manualRequested.getAsBoolean();
  }

  @Override
  public void execute() {
    usedForManual |= manualRequested.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    if (!bumperPressed.getAsBoolean()
        && DriverStation.isTeleopEnabled()
        && !usedForManual
        && !manualRequested.getAsBoolean()) {
      onTap.run();
    }
  }
}
