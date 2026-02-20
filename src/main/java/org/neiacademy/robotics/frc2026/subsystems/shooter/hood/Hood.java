package org.neiacademy.robotics.frc2026.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;

public class Hood extends SubsystemBase {
  @Getter private final String name;
  @Getter private final HoodIO hood;

  private static final double minAngleDegs = Units.degreesToRadians(15);
  private static final double maxAngleDegs = Units.degreesToRadians(45);

  private static double hoodOffsetDegs = 0.0;

  private static LoggedTunableNumber kP;
  private static LoggedTunableNumber kI;
  private static LoggedTunableNumber kD;

  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(String name, HoodIO hood) {
    this.name = name;
    this.hood = hood;

    kP = new LoggedTunableNumber(name + "/kP");
    kI = new LoggedTunableNumber(name + "/kI");
    kD = new LoggedTunableNumber(name + "/kD");

    hood.setPID(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
  }

  public void periodic() {
    hood.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public Command setAngleDegrees(DoubleSupplier angleDegrees) {
    return Commands.runEnd(
        () -> {
          double targetRotations =
              (MathUtil.clamp(angleDegrees.getAsDouble(), minAngleDegs, maxAngleDegs)
                      - hoodOffsetDegs)
                  / 360;
          hood.setPIDSetpoint(targetRotations);
        },
        () -> hood.setOutputPIDZero());
  }

  public void setVelocity(double normalizedVelocity, double feedForward) {
    hood.setVelocity(normalizedVelocity, feedForward);
  }
}
