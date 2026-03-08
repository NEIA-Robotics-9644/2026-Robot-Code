package org.neiacademy.robotics.frc2026.subsystems.intakeroller;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {

  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert intakeRollerMotorDisconnectedAlert =
      new Alert("IntakeRoller motor disconnected!", Alert.AlertType.kWarning);

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);

    intakeRollerMotorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.connected));
  }

  public Command runVoltageCommand(DoubleSupplier volts) {
    return run(() -> io.runVoltage(volts.getAsDouble())).finallyDo(() -> io.stop());
  }

  // always have a timeout when running this
  public Command runAutoVoltageCommand(DoubleSupplier volts) {
    return runEnd(() -> io.runVoltage(volts.getAsDouble()), () -> io.runVoltage(0));
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
