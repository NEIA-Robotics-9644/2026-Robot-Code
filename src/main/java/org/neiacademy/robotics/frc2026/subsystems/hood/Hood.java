package org.neiacademy.robotics.frc2026.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.Constants;
import org.neiacademy.robotics.frc2026.util.Util;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO actuators) {
    super("Hood");
    io = actuators;
  }

  @Override
  public void periodic() {
    io.updateCurrentPosition();
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);
  }

  public Command zeroHood() {
    return Commands.race(
            Commands.waitSeconds(0.75), new InstantCommand(() -> io.setSpeedNormalized(0.25)))
        .andThen(new InstantCommand(() -> io.resetPosition()));
  }

  public Command positionCommand(double position) {
    return runOnce(() -> io.setPositionNormalized(position))
        .andThen(Commands.waitUntil(() -> io.isPositionWithinTolerance()));
  }

  public Command runTrackedPositionCommand(DoubleSupplier position) {
    return run(() -> io.setPositionNormalized(position.getAsDouble()));
  }

  public Command tuckCommand(DoubleSupplier position) {
    return run(() -> io.setPositionNormalized(position.getAsDouble()));
  }

  public boolean atSetpoint() {
    return Util.epsilonEquals(
        inputs.leftNormalizedPosition,
        inputs.leftSetpointPosition,
        Constants.Hood.POSITION_TOLERANCE);
  }
}
