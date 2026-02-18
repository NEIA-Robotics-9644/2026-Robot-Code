package org.neiacademy.robotics.frc2026.subsystems.intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;


public class Intake extends SubsystemBase{
    private final IntakeIO wheel;
    private final IntakeIO pivot;
    private final PIDController pivotFeedback= new PIDController(0, 0, 0);
    private LoggedTunableNumber KP = new LoggedTunableNumber("Pivot/P", 0.1);
    private LoggedTunableNumber KI = new LoggedTunableNumber("Pivot/I", 0.0);
    private LoggedTunableNumber KD = new LoggedTunableNumber("Pivot/P", 0.0);
    //private LoggedTunableNumber pivotG = new LoggedTunableNumber("Pivot/G", 0.1);

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO wheel, IntakeIO pivot) {
        this.wheel = wheel;
        this.pivot = pivot;
        pivotFeedback.setPID(KP.get(), KI.get(), KD.get());
    }
    
    public void periodic() {
        wheel.updateInputs(inputs);
        wheel.periodic();
        Logger.processInputs("IntakeWheel", inputs);
        pivot.updateInputs(inputs);
        pivot.periodic();
        Logger.processInputs("Pivot", inputs);
    }

    public void setVelocity(double velocity) {
        wheel.setVelocity(velocity);
    }

    public void setPivotVelocity(double velocity) {
        pivot.setVelocity(velocity);
    }

    public Command pivotGoToAngle(DoubleSupplier angle){
        var command =
            Commands.runEnd(
                () -> {
                    pivotFeedback.setPID(KP.get(), KI.get(), KD.get());
                    var output = 
                        pivotFeedback.calculate(
                            pivot.getPositionRads(), angle.getAsDouble()
                    );

                    setPivotVelocity(output);
                },
                () -> {
                    setPivotVelocity(0.00);
                }
            )
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            .withName("Pivot Setpoint Control");

        return command;
    }
    
    
}
