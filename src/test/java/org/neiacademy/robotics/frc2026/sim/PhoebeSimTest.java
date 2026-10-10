package org.neiacademy.robotics.frc2026.sim;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;
import org.neiacademy.robotics.frc2026.Presets;

class PhoebeSimTest {
  @Test
  void collectsOnlyWhenDeployedAndRunningThenShootsFiniteInventoryAndStopsWhenDisabled() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    FuelSim world = new FuelSim("FuelSimTest");
    PhoebeSim robot =
        new PhoebeSim(() -> new Pose2d(2, 2, Rotation2d.kZero), ChassisSpeeds::new, world);
    try {
      world.clearFuel();
      world.spawnFuel(new Translation3d(2.7, 2, 0.08), Translation3d.kZero);
      robot.intakeRoller.runVoltage(12);
      robot.update();
      assertEquals(8, robot.getStoredFuel(), "Retracted intake must not collect");
      robot.intakeDeploy.runPosition(Presets.Intake.EXTEND_ANGLE_DEG.get());
      for (int i = 0; i < 20; i++) robot.update();
      assertEquals(9, robot.getStoredFuel());
      assertEquals(0, world.fuels.size());
      // Isolate hopper depletion: do not recapture shots that bounce back into the intake.
      robot.intakeRoller.stop();

      robot.leftShooter.runVelocity(300);
      robot.rightShooter.runVelocity(300);
      robot.loader.runVoltage(-12);
      robot.spindexer.runVoltage(-12);
      for (int i = 0; i < 50; i++) robot.update();
      assertEquals(0, robot.getLaunchedFuel(), "Reverse unjam must not shoot");
      robot.loader.runVoltage(12);
      robot.spindexer.runVoltage(12);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      for (int i = 0; i < 50; i++) robot.update();
      assertEquals(0, robot.getLaunchedFuel());
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      for (int i = 0; i < 200; i++) robot.update();
      assertEquals(9, robot.getLaunchedFuel());
      assertEquals(0, robot.getStoredFuel());
      for (int i = 0; i < 100; i++) robot.update();
      assertEquals(9, robot.getLaunchedFuel(), "Empty hopper cannot create fuel");
      robot.reset();
      assertEquals(8, robot.getStoredFuel());
      assertEquals(0, robot.getLaunchedFuel());
      assertTrue(world.fuels.size() > 100);
    } finally {
      world.stop();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  void launchUsesFrontOfRotatedRobotAndFieldRelativeMotion() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    FuelSim world =
        new FuelSim("FuelSimDirectionTest") {
          @Override
          public void updateSim() {} // Inspect launch before gravity and collisions.
        };
    PhoebeSim robot =
        new PhoebeSim(
            () -> new Pose2d(2, 2, Rotation2d.fromDegrees(90)),
            () -> new ChassisSpeeds(1, 0, 0),
            world);
    try {
      world.clearFuel();
      robot.leftShooter.runVelocity(300);
      robot.rightShooter.runVelocity(300);
      robot.loader.runVoltage(12);
      robot.spindexer.runVoltage(12);
      for (int i = 0; i < 50; i++) robot.update();
      assertFalse(world.fuels.isEmpty());
      var shot = world.fuels.get(0);
      assertEquals(0, shot.vel.getX(), 1e-8);
      assertTrue(
          shot.vel.getY() > 1, "Front +X at 90 degrees shoots along field +Y plus robot velocity");
      assertTrue(shot.vel.getZ() > 0);
      assertEquals(0.65, shot.pos.getZ(), 1e-8);
    } finally {
      world.stop();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }
}
