package org.neiacademy.robotics.frc2026;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeployIO;

class IntakeBindingSimulationTest {
  private static final int LEFT_BUMPER_BUTTON = 5;
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private final RecordingIntakeIO io = new RecordingIntakeIO();

  private void cycles(int count) {
    for (int i = 0; i < count; i++) {
      SimHooks.stepTiming(0.02);
      DriverStationSim.notifyNewData();
      scheduler.run();
    }
  }

  @Test
  void actualUsbZeroBindingHandlesTapHoldReleaseAndIntakeDuringShooting() {
    assertTrue(HAL.initialize(500, 0));
    scheduler.cancelAll();
    scheduler.getDefaultButtonLoop().clear();
    scheduler.unregisterAllSubsystems();
    SimHooks.pauseTiming();
    RobotController.setTimeSource(RobotController::getFPGATime);
    XboxControllerSim driver = new XboxControllerSim(0);
    XboxControllerSim operator = new XboxControllerSim(1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    try {
      assertEquals(Constants.Mode.SIM, Constants.currentMode);
      new RobotContainer(io);
      cycles(5);
      DriverStationSim.setEnabled(true);
      cycles(2);

      // USB 1 LB must not trigger the USB 0 intake action.
      operator.setRawButton(LEFT_BUMPER_BUTTON, true);
      cycles(30);
      assertTrue(io.events.isEmpty());
      operator.setRawButton(LEFT_BUMPER_BUTTON, false);
      cycles(2);

      // 200 ms tap: no pivot motion before release, then deploy and hold voltage.
      driver.setRawButton(LEFT_BUMPER_BUTTON, true);
      cycles(10);
      assertTrue(io.events.isEmpty(), "A short LB tap must not command retraction");
      driver.setRawButton(LEFT_BUMPER_BUTTON, false);
      cycles(1);
      assertEquals("position", io.mode);
      assertEquals(Presets.Intake.EXTEND_ANGLE_DEG.get(), io.value, 1e-9);
      cycles(30);
      assertEquals("voltage", io.mode);
      assertEquals(0.40, io.value, 1e-9);
      System.out.println("SIM PASS: USB 1 isolation; 200 ms tap deploys; release reaches +0.40 V");

      // Hold while shooting: wait at least 0.5 s, then keep alternating beyond 6 s.
      io.events.clear();
      driver.setRightTriggerAxis(1.0);
      driver.setRawButton(LEFT_BUMPER_BUTTON, true);
      double pressedAt = Timer.getTimestamp();
      cycles(24);
      assertTrue(io.positions().isEmpty(), "No agitation before the half-second threshold");
      cycles(3);
      assertFalse(io.positions().isEmpty());
      double delay = io.positions().get(0).time - pressedAt;
      assertTrue(delay >= 0.48 && delay <= 0.56, "Hold start delay: " + delay);
      cycles(375);
      List<Request> positions = io.positions();
      assertTrue(positions.size() > 20, "Holding must keep cycling, not stop at 2.25 or 6 seconds");
      assertTrue(positions.get(positions.size() - 1).time - pressedAt > 7.0);
      for (int i = 1; i < positions.size(); i++) {
        assertNotEquals(positions.get(i - 1).value, positions.get(i).value);
        double interval = positions.get(i).time - positions.get(i - 1).time;
        assertTrue(interval >= 0.24 && interval <= 0.32, "Phase interval: " + interval);
      }
      System.out.println(
          "SIM PASS: hold starts at "
              + delay
              + " s; "
              + positions.size()
              + " alternating position phases while shooting");

      // Intake trigger must not silently kill agitation while LB remains held.
      driver.setLeftTriggerAxis(1.0);
      int before = io.positions().size();
      cycles(60);
      assertTrue(
          io.positions().size() >= before + 3,
          "Pressing intake while LB is held must not permanently cancel agitation");
      driver.setLeftTriggerAxis(0.0);
      cycles(2);

      driver.setRawButton(LEFT_BUMPER_BUTTON, false);
      cycles(1);
      assertEquals("position", io.mode);
      assertEquals(Presets.Intake.EXTEND_ANGLE_DEG.get(), io.value, 1e-9);
      cycles(30);
      assertEquals("voltage", io.mode);
      assertEquals(0.40, io.value, 1e-9);
      driver.setRightTriggerAxis(0.0);
      io.events.clear();

      // Every new hold must qualify again; release immediately cancels the motion.
      driver.setRawButton(LEFT_BUMPER_BUTTON, true);
      cycles(24);
      assertTrue(io.positions().isEmpty());
      cycles(3);
      assertFalse(io.positions().isEmpty());
      driver.setRawButton(LEFT_BUMPER_BUTTON, false);
      cycles(1);
      assertEquals(Presets.Intake.EXTEND_ANGLE_DEG.get(), io.value, 1e-9);
      cycles(30);
      DriverStationSim.setEnabled(false);
      cycles(2);
      assertEquals("stop", io.mode);
      System.out.println(
          "SIM PASS: concurrent intake, release, re-hold threshold, disable cleanup");
    } finally {
      scheduler.cancelAll();
      scheduler.getDefaultButtonLoop().clear();
      scheduler.unregisterAllSubsystems();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      RobotController.setTimeSource(RobotController::getFPGATime);
      SimHooks.resumeTiming();
    }
  }

  private record Request(double time, String mode, double value) {}

  private static class RecordingIntakeIO implements IntakeDeployIO {
    final List<Request> events = new ArrayList<>();
    String mode = "none";
    double value;
    double target;

    private void record(String nextMode, double nextValue) {
      if (!mode.equals(nextMode) || value != nextValue) {
        events.add(new Request(Timer.getTimestamp(), nextMode, nextValue));
      }
      mode = nextMode;
      value = nextValue;
    }

    List<Request> positions() {
      return events.stream().filter(e -> e.mode.equals("position")).toList();
    }

    @Override
    public void updateInputs(IntakeDeployIOInputs inputs) {
      // Record control requests only; this is not a calibrated intake physics model.
      inputs.motorConnected = true;
      inputs.cancoderConnected = true;
      inputs.positionSetpointRads = Units.rotationsToRadians(target);
    }

    @Override
    public void runPosition(double position) {
      target = position;
      record("position", position);
    }

    @Override
    public void runVoltage(double volts) {
      record("voltage", volts);
    }

    @Override
    public void stop() {
      record("stop", 0.0);
    }
  }
}
