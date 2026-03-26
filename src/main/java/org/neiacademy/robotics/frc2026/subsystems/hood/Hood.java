package org.neiacademy.robotics.frc2026.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.neiacademy.robotics.frc2026.Constants;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private LoggedNetworkNumber position = new LoggedNetworkNumber("/Tuning/HoodPosition", 0);

  public Hood(HoodIO actuators) {
    super("Hood");
    io = actuators;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);
  }

  public Command extendHood() {
    return this.run(() -> io.setPositionMillimeters(Constants.Hood.LENGTH_MILLIMETERS))
        .withName("ExtendHood");
  }

  public Command retractHood() {
    return this.run(() -> io.setPositionMillimeters(0)).withName("RetractHood");
  }

  public Command zeroHood() {
    return Commands.race(Commands.waitSeconds(0.75), new InstantCommand(() -> io.setSpeed(0.25)))
        .andThen(new InstantCommand(() -> io.resetPosition()));
  }

  public Command goTo(DoubleSupplier pulseWidth) {
    return this.run(() -> io.setPositionMillimeters((pulseWidth.getAsDouble())))
        .repeatedly()
        .withName("HoodGoToPosition");
  }

  public Command tunableShot() {
    return this.runOnce(() -> io.setPositionMillimeters(position.get()))
        .withName("HoodTunableShot")
        .repeatedly();
  }
}
