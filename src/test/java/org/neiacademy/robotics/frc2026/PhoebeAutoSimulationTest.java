package org.neiacademy.robotics.frc2026;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;
import org.neiacademy.robotics.frc2026.sim.PhoebeSim;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;

class PhoebeAutoSimulationTest {
  @Test
  void compareFuelThroughputWithActualSimulatedModules() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    var scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.unregisterAllSubsystems();
    scheduler.getDefaultButtonLoop().clear();
    SimHooks.pauseTiming();
    RobotController.setTimeSource(RobotController::getFPGATime);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    try {
      com.pathplanner.lib.path.PathPlannerPath.clearCache();
      var container = new RobotContainer();
      var sf = RobotContainer.class.getDeclaredField("phoebeSim");
      sf.setAccessible(true);
      var sim = (PhoebeSim) sf.get(container);
      var df = RobotContainer.class.getDeclaredField("drive");
      df.setAccessible(true);
      var drive = (Drive) df.get(container);
      for (String name :
          new String[] {
            "Right NERD Safe Bump",
            "Right NZ Steal And Shoot Auto",
            "Right NZ Bump No Cross Wait Steal and Shoot Auto"
          }) {
        sim.reset();
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        var auto = new PathPlannerAuto(name);
        auto.schedule();
        int cycles = name.startsWith("Right NERD") ? 1000 : 1500;
        int launchedAtTwenty = 0;
        for (int i = 0; i < cycles; i++) {
          SimHooks.stepTiming(0.02);
          DriverStationSim.notifyNewData();
          scheduler.run();
          container.simulationPeriodic();
          if (i == 999) launchedAtTwenty = sim.getLaunchedFuel();
          if (i % 50 == 0)
            System.out.printf(
                "FUEL %s %.2fs stored=%d launched=%d pose=%s%n",
                name, i * .02, sim.getStoredFuel(), sim.getLaunchedFuel(), drive.getPose());
        }
        System.out.printf(
            "THROUGHPUT %s: at20=%d total=%d remaining=%d%n",
            name, launchedAtTwenty, sim.getLaunchedFuel(), sim.getStoredFuel());
        assertTrue(
            sim.getLaunchedFuel() > 0,
            name + " must launch fuel through the actual simulated mechanisms");
        auto.cancel();
        scheduler.cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        for (int i = 0; i < 10; i++) {
          SimHooks.stepTiming(.02);
          scheduler.run();
          container.simulationPeriodic();
        }
      }
    } finally {
      scheduler.cancelAll();
      scheduler.unregisterAllSubsystems();
      scheduler.getDefaultButtonLoop().clear();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      SimHooks.resumeTiming();
    }
  }
}
