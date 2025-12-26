[![CI](https://github.com/NEIA-Robotics-9644/2026-Robot-Code/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/NEIA-Robotics-9644/2026-Robot-Code/actions/workflows/main.yml)
![Static Badge](https://img.shields.io/badge/FRC-Team_9644-cb007b?logo=first&labelColor=gray&link=https%3A%2F%2Fwww.thebluealliance.com%2Fteam%2F9644)
<!-- ![Static Badge](https://img.shields.io/badge/WPI-Lib-%234d4848?style=flat-square&labelColor=8c150d)
![Static Badge](https://img.shields.io/badge/Photon-Vision-ffffff?style=flat-square&labelColor=006492)
![Static Badge](https://img.shields.io/badge/Path-Planner-ffffff?style=flat-square&labelColor=%232c3aad)
![Static Badge](https://img.shields.io/badge/Advantage-Scope-dee3ff?style=flat-square&labelColor=0027e6)
![Static Badge](https://img.shields.io/badge/Advantage-Kit-fff3d9?style=flat-square&labelColor=fec007) -->

# 2026-Robot-Code
2026 REBUILT&trade; Code Base

<p align="center">
  <img src="assets/logo.png" width="50%" height="50%">
</p>

## Repository and Contribution Rules
The ```main``` branch is protected.
Do not attempt to commit or push directly to ```main```.
Before starting work:
Pull the latest updates to ```main```, then create a new branch and name your branch clearly. For example:
```subsystem-arm-pid-tuning```
```bugfix-xbox-button-not-working```
```docs-update-readme```

Once your work is complete, open a new Pull Request and check against this list:
- Add vishnu09bharath as a reviewer.
  - Optionally, request the `software-leads` team to review. All pull requests must be approved by only vishnu09bharath, but feedback/improvement suggestions may be delegated to other software leads.
- Code has thorough documentation in the form of comments, or where necessary, in the ```README```.
- All public methods include method descriptions for intellisense where useful (custom utils, state params, etc.).
- The code passes the [FRC 190 Code Standards](https://team-190.github.io/190-Robot-Code-Standards/ROBOTSTATE_STANDARDS) for the relevant type (subsystem, command, etc.) and all global standards.
- Code inspired by other teams/examples is credited for future reference.
- The latest commit passes all build checks.
- All merge conflicts are resolved.

### During Competitions
All competitions, whether pre-season, season, or off-season, will be given their own branch designated with the event code and year. Ex: ```event_MAWOR2025```. These branches are unprotected and have no rulesets associated with them. All code deployed to the competition robot will be sourced from this designated branch. This allows any member to push code without requiring a pull request, allowing for faster, hassle-free changes to robot code. After each competition, the branch will be merged with ```main``` without squashing commits, given it is in a working state.  

Note: The prefix ```event``` will ensure that any code deployed to the robot will be committed to the branch (see [build.gradle](https://github.com/NEIA-Robotics-9644/2026-Robot-Code/blob/d5408845ff7be9e6b541cbb27b5af825ab436651/build.gradle#L156-L186)). This allows for deterministic log replay in AdvantageScope, detailed [here](https://docs.advantagekit.org/getting-started/replay-watch), given that we know what code was running on the robot at anytime by extracting the commit hash from the log file metadata.

### Additional Notes

#### Package Structure
The codebase follows a clean separation between robot-specific code (`org.neiacademy.robotics.frc2025.*`) and reusable library code (`org.neiacademy.robotics.lib.*`). The library packages provide abstractions and utilities that could be reused across different robot projects. This structure is **_heavily_** inspired by Team 3467: Windham Windup. See their [W8 Library](https://github.com/WHS-FRC-3467/W8-Library/tree/dev) for more details.
