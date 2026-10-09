package org.neiacademy.robotics.frc2026.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.junit.jupiter.api.Test;
import org.neiacademy.robotics.frc2026.commands.ManualPivotBumperCommand;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.*;

class ManualPivotTest {
  @Test
  void integratesFromMeasurementClampsAndHoldsDespiteSensorLag() {
    assertTrue(HAL.initialize(500, 0));
    double[] target = {Double.NaN};
    IntakeDeploy pivot =
        new IntakeDeploy(
            new IntakeDeployIO() {
              @Override
              public void updateInputs(IntakeDeployIOInputs inputs) {
                inputs.rotorPositionRads = Units.rotationsToRadians(0.25);
              }

              @Override
              public void runPosition(double position) {
                target[0] = position;
              }
            });
    pivot.periodic();
    double[] stick = {0.05};
    Command command = pivot.manualPositionCommand(() -> stick[0]);
    command.initialize();
    command.execute();
    assertEquals(0.25, target[0], 1e-9);
    stick[0] = 1;
    for (int i = 0; i < 15; i++) command.execute();
    assertEquals(0.35, target[0], 1e-9);
    stick[0] = 0;
    for (int i = 0; i < 15; i++) command.execute();
    assertEquals(0.35, target[0], 1e-9);
    stick[0] = 1;
    for (int i = 0; i < 100; i++) command.execute();
    assertEquals(0.5, target[0], 1e-9);
    stick[0] = -1;
    for (int i = 0; i < 100; i++) command.execute();
    assertEquals(0, target[0], 1e-9);
    command.end(true);
    CommandScheduler.getInstance().unregisterSubsystem(pivot);
  }

  @Test
  void bumperTapIsDeferredAndEitherManualButtonOrderSuppressesIt() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    var scheduler = CommandScheduler.getInstance();
    boolean[] buttons = {false, false};
    int[] taps = {0};
    new Trigger(() -> buttons[0])
        .whileTrue(
            new ManualPivotBumperCommand(() -> buttons[0], () -> buttons[1], () -> taps[0]++));
    try {
      buttons[0] = true;
      scheduler.run();
      assertEquals(0, taps[0]);
      buttons[0] = false;
      scheduler.run();
      assertEquals(1, taps[0]);
      for (boolean stickFirst : new boolean[] {true, false}) {
        buttons[1] = stickFirst;
        scheduler.run();
        buttons[0] = true;
        scheduler.run();
        buttons[1] = true;
        scheduler.run();
        buttons[1] = false;
        scheduler.run();
        buttons[0] = false;
        scheduler.run();
        assertEquals(1, taps[0]);
      }
      buttons[0] = true;
      scheduler.run();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      scheduler.run();
      assertEquals(1, taps[0]);
    } finally {
      scheduler.cancelAll();
      scheduler.getDefaultButtonLoop().clear();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }
}
