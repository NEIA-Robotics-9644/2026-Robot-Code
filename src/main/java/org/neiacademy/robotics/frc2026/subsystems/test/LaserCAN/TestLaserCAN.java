package org.neiacademy.robotics.frc2026.subsystems.test.LaserCAN;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.lib.genericIO.GenericLaserCANSubsystem.GenericLaserCANSubsystem;
import org.neiacademy.robotics.lib.genericIO.GenericLaserCANSubsystem.GenericLaserCANSubsystem.DistanceState;

public class TestLaserCAN extends GenericLaserCANSubsystem<TestLaserCAN.State> {

  public Trigger triggered = new Trigger(() -> super.isTriggered());

  private Debouncer validDebouncer =
      new Debouncer(2, DebounceType.kRising); // need to test how changing this value affects things

  public Trigger validMeasurement =
      new Trigger(() -> validDebouncer.calculate(io.getValidStatus()));

  @RequiredArgsConstructor
  @Getter
  public enum State implements DistanceState {
    DEFAULT(
        Distance.ofBaseUnits(
            0.05, Meter)); // also need to figure out what this value does - assume the distance to
    // trigger

    @Getter private final Distance distance;
  }

  @Getter @Setter private State state = State.DEFAULT;

  public TestLaserCAN(TestLaserCANIO io) {
    super(TestLaserCANConstants.kSubSysConstants.kName, io);
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput("TestLaserCan/Fallback Active", !validMeasurement.getAsBoolean());
  }
}
