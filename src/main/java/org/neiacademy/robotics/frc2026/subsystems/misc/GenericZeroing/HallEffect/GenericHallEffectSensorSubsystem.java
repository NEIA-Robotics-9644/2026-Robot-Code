package org.neiacademy.robotics.frc2026.subsystems.misc.GenericZeroing.HallEffect;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GenericHallEffectSensorSubsystem extends SubsystemBase {

  private final GenericHallEffectSensorIO io;

  // IMPORTANT: use the AutoLogged version
  private final HallEffectSensorIOInputsAutoLogged inputs;

  private final String name;

  public GenericHallEffectSensorSubsystem(String name, GenericHallEffectSensorIO io) {
    this.inputs = new HallEffectSensorIOInputsAutoLogged();
    this.name = name;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("HallEffectSensors/" + name, inputs);
    Logger.recordOutput("HallEffectSensors/" + name + "/Triggered", inputs.triggered);
  }

  public boolean isTriggered() {
    return inputs.triggered;
  }

  public boolean isValid() {
    return io.getValidStatus();
  }
}
