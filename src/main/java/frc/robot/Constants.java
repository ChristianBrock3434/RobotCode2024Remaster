// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final CommandXboxController driver = new CommandXboxController(0);

  public static double farShotDistance = 4.4; // meters

  public static final double loopPeriodSecs = 0.02;
  private static RobotType robotType = RobotType.COMPBOT;
  public static final boolean tuningMode = false;

  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    DEVBOT,
    COMPBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType);
      System.exit(1);
    }
  }

  public final class ActuationConstants {
    public static final double actuationInternalGearRatio = 8;
    public static final double actuationGearRatio = 2;

    // 360 degrees per rotation, 8:1 gear ratio
    public static final double actuationTicksPerDegree = 1.0 / 360.0 * actuationGearRatio;
    public static final double actuationInternalTicksPerDegree =
        1.0 / 360.0 * actuationInternalGearRatio;

    public static final double actuationStartPosition = -65;
    public static double actuationPickUpPosition = 98; // 100
    public static final double actuationTuckPosition = -63.3;

    public static final double actuationOffset = 1.9 - actuationStartPosition;
  }

  public final class IntakeConstants {
    public static final double intakeGearRatio = 1.3333333333333333333;

    public static final double intakeVoltage = 7;
    public static final double autoIntakeVoltage = 7;

    public static final double feedVelocity = 60;
    public static final double feedAcceleration = 102;

    public static final double intakeVelocity = 15;
    public static final double intakeAcceleration = 40;

    public static final double outtakeVelocity = -15;
    public static final double outtakeAcceleration = 40;
  }

  public final class IndexerConstants {
    public static final double indexerVelocity = 60; // 40 amp //60 shooting
    public static final double indexerAcceleration = 100;
  }

  public final class AngleControllerConstants {
    public static final double angleTicksPerDegree =
        (11.78 * 1.75 / 360 * 4) / 1.5; // Estimate given by Adam, ask him how to do it if you need

    public static final double angleStartingPosition = 0;
    public static final double angleRestingPosition = 15;

    public static final double trapAngle = 0;
    public static double ampAngle = 0;

    public static double subwooferShotAngle = -1.25;
    public static double podiumShotAngle = 23.0; // 23 Half Line Shot 34.75
    public static double chainShotAngle = 33.25;
    public static double championshipShotAngle = 33;
    public static double passShotAngle = 15;
  }

  public final class ShooterConstants {
    public static final double shooterSequenceAcceleration = 100;

    public static final double trapSpeed = 36;
    public static double ampSpeed = 8.5;

    public static double subwooferShotSpeed = 45; // 27
    public static double podiumShotSpeed = 44; // 55 Half Line Shot 62
    public static double chainShotSpeed = 54; // 44
    public static double championshipShotSpeed = 52;
    public static double passShotSpeed = 42.5; // 30
  }

  public final class ClimberConstants {
    public static final double minClimberHeight = 0;
    public static final double maxClimberHeight = 240;
  }

  public final class SlapperConstants {
    public static final double slapperOffset = 99.941;

    public static final double slapperGearRatio = 2.25;
    public static final double slapperTicksPerDegree = (1.0 / 360.0);

    public static final double slapperStartingPosition = 0;
    public static final double slapperRestingPosition = 225;
    public static final double slapperAmpPosition = 127.5;
    public static final double slapperPushNotePosition = 150;
    public static final double slapperPostAmpPosition = 120;
  }

  public final class TurningConstants {
    public static double championshipShotOffset = 4.5;
    public static double chainShotOffset = 3;
    public static double podiumShotOffset = 5;
    public static double passShotOffset = 5;

    public static double championshipShotManualRot = 151;
    public static double chainShotManualRot = -180;
    public static double podiumShotManualRot = 165;
    public static double passShotManualRot = 160;
  }
}
