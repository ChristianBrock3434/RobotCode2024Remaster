// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import frc.robot.subsystems.drive.controllers.AutoAlignController;
import frc.robot.subsystems.drive.controllers.AutoDriveController;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.subsystems.drive.controllers.SimpleDriveController;
import frc.robot.subsystems.drive.controllers.TeleopDriveController;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.swerve.ModuleLimits;
import frc.robot.util.swerve.SwerveSetpoint;
import frc.robot.util.swerve.SwerveSetpointGenerator;
import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final LoggedTunableNumber coastWaitTime =
      new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecThreshold =
      new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", 0.05);

  public enum DriveMode {
    /** Driving with input from driver joysticks. (Default) */
    TELEOP,

    /** Driving with chassis speeds from pathplanner in auto */
    AUTO,

    /** Driving to a location on the field automatically. */
    AUTO_ALIGN,

    /** Characterizing (modules oriented forwards, motor outputs supplied externally). */
    CHARACTERIZATION,

    /** Running wheel radius characterization routine (spinning in circle) */
    WHEEL_RADIUS_CHARACTERIZATION,

    /** Drive output directly to wheels, controlled like teleop */
    SIMPLE
  }

  public enum CoastRequest {
    AUTOMATIC,
    ALWAYS_BRAKE,
    ALWAYS_COAST
  }

  @AutoLog
  public static class OdometryTimestampInputs {
    public double[] timestamps = new double[] {};
  }

  public static final Lock odometryLock = new ReentrantLock();
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);

  private final OdometryTimestampInputsAutoLogged odometryTimestampInputs =
      new OdometryTimestampInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];

  // Store previous positions and time for filtering odometry data
  private SwerveDriveWheelPositions lastPositions = null;
  private double lastTime = 0.0;

  /** Active drive mode. */
  private DriveMode currentDriveMode = DriveMode.TELEOP;

  private double characterizationInput = 0.0;
  private boolean modulesOrienting = false;
  private final Timer lastMovementTimer = new Timer();

  @AutoLogOutput(key = "Drive/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  @AutoLogOutput(key = "Drive/CoastRequest")
  public CoastRequest coastRequest = CoastRequest.AUTOMATIC;

  private boolean lastEnabled = false;

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private final SwerveSetpointGenerator setpointGenerator;

  private final TeleopDriveController teleopDriveController;
  private final AutoDriveController autoDriveController;
  private final SimpleDriveController simpleDriveController;
  private AutoAlignController autoAlignController = null;
  private HeadingController headingController = null;

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);
    lastMovementTimer.start();
    setBrakeMode(true);

    setpointGenerator =
        new SwerveSetpointGenerator(DriveConstants.kinematics, DriveConstants.moduleTranslations);
    teleopDriveController = new TeleopDriveController();
    autoDriveController = new AutoDriveController();
    simpleDriveController = new SimpleDriveController();

    configurePathPlanner();
  }

  public void periodic() {
    // Update & process inputs
    odometryLock.lock();
    // Read timestamps from odometry thread and fake sim timestamps
    odometryTimestampInputs.timestamps =
        timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    if (odometryTimestampInputs.timestamps.length == 0) {
      odometryTimestampInputs.timestamps = new double[] {Timer.getFPGATimestamp()};
    }
    timestampQueue.clear();
    Logger.processInputs("Drive/OdometryTimestamps", odometryTimestampInputs);
    // Read inputs from gyro
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    // Read inputs from modules
    Arrays.stream(modules).forEach(Module::updateInputs);
    odometryLock.unlock();

    ModuleLimits currentModuleLimits = RobotState.getInstance().getModuleLimits();

    // Calculate the min odometry position updates across all modules
    int minOdometryUpdates =
        IntStream.of(
                odometryTimestampInputs.timestamps.length,
                Arrays.stream(modules)
                    .mapToInt(module -> module.getModulePositions().length)
                    .min()
                    .orElse(0))
            .min()
            .orElse(0);
    if (gyroInputs.connected) {
      minOdometryUpdates = Math.min(gyroInputs.odometryYawPositions.length, minOdometryUpdates);
    }
    // Pass odometry data to robot state
    for (int i = 0; i < minOdometryUpdates; i++) {
      int odometryIndex = i;
      Rotation2d yaw = gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null;
      // Get all four swerve module positions at that odometry update
      // and store in SwerveDriveWheelPositions object
      SwerveDriveWheelPositions wheelPositions =
          new SwerveDriveWheelPositions(
              Arrays.stream(modules)
                  .map(module -> module.getModulePositions()[odometryIndex])
                  .toArray(SwerveModulePosition[]::new));
      // Filtering based on delta wheel positions
      boolean includeMeasurement = true;
      if (lastPositions != null) {
        double dt = odometryTimestampInputs.timestamps[i] - lastTime;
        for (int j = 0; j < modules.length; j++) {
          double velocity =
              (wheelPositions.positions[j].distanceMeters
                      - lastPositions.positions[j].distanceMeters)
                  / dt;
          double omega =
              wheelPositions.positions[j].angle.minus(lastPositions.positions[j].angle).getRadians()
                  / dt;
          // Check if delta is too large
          if (Math.abs(omega) > currentModuleLimits.maxSteeringVelocity() * 5.0
              || Math.abs(velocity) > currentModuleLimits.maxDriveVelocity() * 5.0) {
            includeMeasurement = false;
            break;
          }
        }
      }
      // If delta isn't too large we can include the measurement.
      if (includeMeasurement) {
        lastPositions = wheelPositions;
        RobotState.getInstance()
            .addOdometryObservation(
                new RobotState.OdometryObservation(
                    wheelPositions, yaw, odometryTimestampInputs.timestamps[i]));
        lastTime = odometryTimestampInputs.timestamps[i];
      }
    }

    // Update current velocities use gyro when possible
    ChassisSpeeds robotRelativeVelocity = getSpeeds();
    robotRelativeVelocity.omegaRadiansPerSecond =
        gyroInputs.connected
            ? gyroInputs.yawVelocityRadPerSec
            : robotRelativeVelocity.omegaRadiansPerSecond;
    RobotState.getInstance()
        .addVelocityData(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond,
                robotRelativeVelocity.vyMetersPerSecond,
                robotRelativeVelocity.omegaRadiansPerSecond));

    // Update brake mode
    // Reset movement timer if moved
    if (Arrays.stream(modules)
        .anyMatch(
            module ->
                Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecThreshold.get())) {
      lastMovementTimer.reset();
    }
    // if (DriverStation.isEnabled() && !lastEnabled) {
    //   coastRequest = CoastRequest.AUTOMATIC;
    // }

    lastEnabled = DriverStation.isEnabled();
    switch (coastRequest) {
      case AUTOMATIC -> {
        if (DriverStation.isEnabled()) {
          setBrakeMode(true);
        } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
          setBrakeMode(false);
        }
      }
      case ALWAYS_BRAKE -> {
        setBrakeMode(true);
      }
      case ALWAYS_COAST -> {
        setBrakeMode(false);
      }
    }

    // Run drive based on current mode
    ChassisSpeeds teleopSpeeds = teleopDriveController.update();
    switch (currentDriveMode) {
      case TELEOP -> {
        // Plain teleop drive
        desiredSpeeds = teleopSpeeds;
        // Add auto aim if present
        if (headingController != null) {
          desiredSpeeds.omegaRadiansPerSecond = headingController.update();
        }
      }
      case AUTO -> {
        // Run auto drive with drive input
        desiredSpeeds = autoDriveController.update();
      }
      case AUTO_ALIGN -> {
        // Run auto align with drive input
        desiredSpeeds = autoAlignController.update();
      }
      case CHARACTERIZATION -> {
        // Run characterization
        for (Module module : modules) {
          module.runCharacterization(0.0, characterizationInput);
        }
      }
      case WHEEL_RADIUS_CHARACTERIZATION -> {
        desiredSpeeds = new ChassisSpeeds(0, 0, characterizationInput);
      }
      case SIMPLE -> {
        desiredSpeeds = simpleDriveController.update();
      }
      default -> {}
    }

    // Run modules
    if (currentDriveMode != DriveMode.CHARACTERIZATION && !modulesOrienting) {
      // Run robot at desiredSpeeds
      // Generate feasible next setpoint
      currentSetpoint =
          setpointGenerator.generateSetpoint(
              currentModuleLimits, currentSetpoint, desiredSpeeds, Constants.loopPeriodSecs);
      SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
      SwerveModuleState[] optimizedSetpointTorques = new SwerveModuleState[4];

      for (int i = 0; i < modules.length; i++) {
        // Optimize setpoints
        optimizedSetpointStates[i] =
            SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], modules[i].getAngle());

        optimizedSetpointTorques[i] = new SwerveModuleState(0.0, optimizedSetpointStates[i].angle);

        modules[i].runSetpoint(optimizedSetpointStates[i], optimizedSetpointTorques[i]);
      }
      Logger.recordOutput("Drive/SwerveStates/Setpoints", optimizedSetpointStates);
      Logger.recordOutput("Drive/SwerveStates/Torques", optimizedSetpointTorques);
    }

    if (DriveConstants.shouldPrintZeros) {
      printZeros();
    }

    // Log chassis speeds and swerve states
    Logger.recordOutput(
        "Drive/SwerveStates/Desired(b4 Poofs)",
        DriveConstants.kinematics.toSwerveModuleStates(desiredSpeeds));
    Logger.recordOutput("Drive/DesiredSpeeds", desiredSpeeds);
    Logger.recordOutput("Drive/SetpointSpeeds", currentSetpoint.chassisSpeeds());
    Logger.recordOutput("Drive/DriveMode", currentDriveMode);
  }

  /** Configure the path planner for the swerve drivetrain */
  private void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
        () -> RobotState.getInstance().getEstimatedPose(), // Supplier of current robot pose
        (pose2D) ->
            RobotState.getInstance().resetPose(pose2D), // Consumer for seeding pose against auto
        () -> autoDriveController.update(), // Supplier of ChassisSpeeds for the robot
        (speeds) -> acceptAutoInput(speeds), // Consumer of ChassisSpeeds to drive the robot
        getPathFollowerConfig(),
        () -> false,
        this); // Subsystem for requirements
  }

  public HolonomicPathFollowerConfig getPathFollowerConfig() {
    return new HolonomicPathFollowerConfig(
        new PIDConstants(2, 0, 0),
        new PIDConstants(2, 0, 0),
        DriveConstants.moduleLimitsFree.maxDriveVelocity(),
        DriveConstants.driveConfig.driveBaseRadius(),
        new ReplanningConfig());
  }

  /**
   * Get an auto based off of the name
   *
   * @param pathName the name of the path
   * @return a command that will run the path
   */
  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /**
   * Get the end state of a path
   *
   * @param autoName the name of the auto to grab from
   * @param index the index of the path in that auto
   * @return the end state of the path
   */
  public State getEndPath(String autoName, int index) {
    var pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    var path = pathGroup.get(index);

    ChassisSpeeds startingChassisSpeed = new ChassisSpeeds(0, 0, 0);
    Rotation2d rot;

    try {
      var previousPath = pathGroup.get(index - 1);
      var startingState = previousPath.getGoalEndState();
      rot = startingState.getRotation();
    } catch (IndexOutOfBoundsException e) {
      rot = RobotState.getInstance().getEstimatedPose().getRotation();
    }

    var trajectory = new PathPlannerTrajectory(path, startingChassisSpeed, rot);
    return trajectory.getEndState();
  }

  /** Pass controller input into teleopDriveController in field relative input */
  public void acceptTeleopInput(
      double controllerX, double controllerY, double controllerOmega, boolean robotRelative) {
    if (DriverStation.isTeleopEnabled()) {
      if (currentDriveMode != DriveMode.AUTO_ALIGN) {
        currentDriveMode = DriveMode.TELEOP;
      }
      teleopDriveController.acceptDriveInput(
          controllerX, controllerY, controllerOmega, robotRelative);
    }
  }

  /** Pass ChassisSpeeds input into autoDriveController in field relative input */
  public void acceptAutoInput(ChassisSpeeds chassisSpeeds) {
    if (DriverStation.isAutonomousEnabled()) {
      currentDriveMode = DriveMode.AUTO;
      autoDriveController.acceptDriveInput(chassisSpeeds);
    }
  }

  public void acceptSimpleInput(double x, double y, double omega, boolean robotRelative) {
    currentDriveMode = DriveMode.SIMPLE;
    simpleDriveController.acceptDriveInput(x, y, omega, robotRelative);
  }

  public ChassisSpeeds getSimpleSpeeds() {
    return simpleDriveController.update();
  }

  public void clearAutoInput() {
    autoDriveController.acceptDriveInput(new ChassisSpeeds());
  }

  /** Sets the goal pose for the robot to drive to */
  public void setAutoAlignGoal(
      Supplier<Pose2d> poseSupplier,
      Supplier<Translation2d> feedforwardSupplier,
      boolean slowMode) {
    if (DriverStation.isEnabled()) {
      currentDriveMode = DriveMode.AUTO_ALIGN;
      autoAlignController = new AutoAlignController(poseSupplier, feedforwardSupplier, slowMode);
    }
  }

  /** Clears the current auto align goal. */
  public void clearAutoAlignGoal() {
    autoAlignController = null;
    currentDriveMode = DriveMode.TELEOP;
  }

  /** Returns true if the robot is at current goal pose. */
  @AutoLogOutput(key = "Drive/AutoAlignCompleted")
  public boolean isAutoAlignGoalCompleted() {
    return autoAlignController == null || autoAlignController.atGoal();
  }

  /** Enable auto aiming on drive */
  public void setHeadingGoal(Supplier<Rotation2d> goalHeadingSupplier) {
    headingController = new HeadingController(goalHeadingSupplier);
  }

  /** Disable auto aiming on drive */
  public void clearHeadingGoal() {
    headingController = null;
  }

  /** Returns true if robot is aimed at speaker */
  @AutoLogOutput(key = "Drive/AtHeadingGoal")
  public boolean atHeadingGoal() {
    return headingController == null || headingController.atGoal();
  }

  /** Runs forwards at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    currentDriveMode = DriveMode.CHARACTERIZATION;
    characterizationInput = input;
  }

  /** Disables the characterization mode. */
  public void endCharacterization() {
    currentDriveMode = DriveMode.TELEOP;
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Runs in a circle at omega. */
  public void runWheelRadiusCharacterization(double omegaSpeed) {
    currentDriveMode = DriveMode.WHEEL_RADIUS_CHARACTERIZATION;
    characterizationInput = omegaSpeed;
  }

  /** Get the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(Module::getPositionRads).toArray();
  }

  /** Set brake mode to {@code enabled} doesn't change brake mode if already set. */
  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }

  /**
   * Returns command that orients all modules to {@code orientation}, ending when the modules have
   * rotated.
   */
  public Command orientModules(Rotation2d orientation) {
    return orientModules(new Rotation2d[] {orientation, orientation, orientation, orientation});
  }

  /**
   * Returns command that orients all modules to {@code orientations[]}, ending when the modules
   * have rotated.
   */
  public Command orientModules(Rotation2d[] orientations) {
    return run(() -> {
          SwerveModuleState[] states = new SwerveModuleState[4];
          for (int i = 0; i < orientations.length; i++) {
            modules[i].runSetpoint(
                new SwerveModuleState(0.0, orientations[i]),
                new SwerveModuleState(0.0, new Rotation2d()));
            states[i] = new SwerveModuleState(0.0, modules[i].getAngle());
          }
          currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), states);
        })
        .until(
            () ->
                Arrays.stream(modules)
                    .allMatch(
                        module ->
                            EqualsUtil.epsilonEquals(
                                module.getAngle().getDegrees(),
                                module.setpointState.angle.getDegrees(),
                                2.0)))
        .beforeStarting(() -> modulesOrienting = true)
        .finallyDo(() -> modulesOrienting = false)
        .withName("Orient Modules");
  }

  public void printZeros() {
    ModuleConfig[] configs = DriveConstants.moduleConfigs;
    Rotation2d[] rots = getAbsoluteModuleRotations();
    for (int i = 0; i < configs.length; i++) {
      System.out.printf(
          "new ModuleConfig(%d, %d, %d, new Rotation2d(%f), %b)",
          configs[i].driveID(),
          configs[i].turnID(),
          configs[i].absoluteEncoderChannel(),
          rots[i].getRadians(),
          configs[i].turnMotorInverted());
      if (i == configs.length - 1) {
        System.out.println();
      } else {
        System.out.println(",");
      }
    }
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  /** Returns the measured speeds of the robot in the robot's frame of reference. */
  @AutoLogOutput(key = "Drive/MeasuredSpeeds")
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
  }

  public Rotation2d[] getAbsoluteModuleRotations() {
    Rotation2d[] rots = new Rotation2d[modules.length];
    for (int i = 0; i < modules.length; i++) {
      rots[i] = modules[i].getAngle().plus(DriveConstants.moduleConfigs[i].absoluteEncoderOffset());
    }
    return rots;
  }

  public static Rotation2d[] getStraightOrientations() {
    return IntStream.range(0, 4).mapToObj(Rotation2d::new).toArray(Rotation2d[]::new);
  }

  public static Rotation2d[] getXOrientations() {
    return Arrays.stream(DriveConstants.moduleTranslations)
        .map(Translation2d::getAngle)
        .toArray(Rotation2d[]::new);
  }

  public static Rotation2d[] getCircleOrientations() {
    return Arrays.stream(DriveConstants.moduleTranslations)
        .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
        .toArray(Rotation2d[]::new);
  }
}
