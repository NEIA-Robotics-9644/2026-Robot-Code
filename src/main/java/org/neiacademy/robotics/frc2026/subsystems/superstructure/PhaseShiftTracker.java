package org.neiacademy.robotics.frc2026.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.neiacademy.robotics.frc2026.GlobalRobotState;

public class PhaseShiftTracker {

  @AutoLogOutput @Getter private Alliance phaseShiftHubStartingAlliance;

  @AutoLogOutput @Getter private Alliance currentActiveAlliance;

  @AutoLogOutput @Getter private Phase currentPhase = Phase.AUTO;

  private double phaseStart;

  @AutoLogOutput @Getter private int alliancePhaseIndex = 0;

  public PhaseShiftTracker() {
    this.phaseStart = Timer.getFPGATimestamp();
  }

  public void periodic() {
    switch (currentPhase) {
      case AUTO -> {
        if (RobotState.isTeleop()) {
          alliancePhaseIndex = 0;
          currentPhase = Phase.TRANSITION;
          phaseStart = Timer.getFPGATimestamp();
        }
      }
      case TRANSITION -> {
        if (Timer.getFPGATimestamp() - phaseStart >= 10) {
          currentPhase = Phase.ALLIANCE_SHIFTS;
          phaseStart = Timer.getFPGATimestamp();
        }
      }
      case ALLIANCE_SHIFTS -> {
        if (Timer.getFPGATimestamp() - phaseStart >= 25) {
          alliancePhaseIndex++;
          phaseStart = Timer.getFPGATimestamp();

          if (alliancePhaseIndex >= 4) currentPhase = Phase.ENDGAME;
        }
      }
      case ENDGAME -> {}
    }

    if (currentPhase == Phase.ALLIANCE_SHIFTS) {
      if (alliancePhaseIndex % 2 == 0) {
        currentActiveAlliance = phaseShiftHubStartingAlliance;
      } else {
        currentActiveAlliance =
            phaseShiftHubStartingAlliance == Alliance.Red ? Alliance.Blue : Alliance.Red;
      }
    }
  }

  /*
      Gets the current alliance of the robot and set
      phaseShiftHubStartingAlliance to it or the opposing
      aliance as long as it exists.
  */
  public void setPhaseShiftUsingCurrentAlliance(boolean useCurrent) {
    phaseShiftHubStartingAlliance =
        useCurrent
            ? GlobalRobotState.getInstance().getAlliance()
            : (GlobalRobotState.getInstance().getAlliance() == Alliance.Red
                ? Alliance.Blue
                : Alliance.Red);
  }
  /*
      Sets the initial state of the hub, denoting whether the initial
      alliance with an active alliance is red or blue.
  */
  public void setHubStartState(Alliance alliance) {
    phaseShiftHubStartingAlliance = alliance;
  }

  public enum Phase {
    AUTO,
    TRANSITION,
    ALLIANCE_SHIFTS,
    ENDGAME
  }
}
