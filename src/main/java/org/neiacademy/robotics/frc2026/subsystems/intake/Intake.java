package org.neiacademy.robotics.frc2026.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  private final IntakeIO wheels;
  private final IntakeIO pivot;

  private static final double minAngleDegs = Units.degreesToRadians(15);
  private static final double maxAngleDegs = Units.degreesToRadians(45);

  private LoggedTunableNumber pivotOffsetDegrees =
      new LoggedTunableNumber("Pivot/OffsetDegrees", 0.1);

  private LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/P", 0.1);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/I", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/D", 0.0);

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO wheels, IntakeIO pivot) {
    this.wheels = wheels;
    this.pivot = pivot;
  }

  public void periodic() {
    wheels.updateInputs(inputs);
    Logger.processInputs("IntakeWheels", inputs);
    pivot.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    pivot.setPID(kP.get(), kI.get(), kD.get());
  }

  public void setWheelsVelocity(double velocity, double feedForward) {
    wheels.setVelocity(velocity, feedForward);
  }

  public void setPivotVelocity(double normalizedVelocity, double feedForward) {
    pivot.setVelocity(normalizedVelocity, feedForward);
  }

  public Command setPivotAngle(DoubleSupplier angleDegrees) {
    return Commands.runEnd(
        () -> {
          double targetRotations =
              (MathUtil.clamp(angleDegrees.getAsDouble(), minAngleDegs, maxAngleDegs)
                      - pivotOffsetDegrees.get())
                  / 360;
          pivot.setPIDSetpoint(targetRotations);
        },
        () -> pivot.setOutputPIDZero());
  }
}
