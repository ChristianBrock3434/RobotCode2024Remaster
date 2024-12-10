// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Actuation;
import frc.robot.subsystems.AngleController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Slapper;
import frc.robot.subsystems.drive.Drive;

/**
 * The Subsystems class represents the collection of subsystems used in the robot. It provides
 * static references to various subsystem objects that are used in the robot.
 */
public final class Subsystems {
  public static final Intake intake = new Intake(); // My intake
  public static final Actuation actuation = new Actuation(); // My actuation
  public static final Shooter shooter = new Shooter(); // My shooter
  public static final Indexer indexer = new Indexer(); // My indexer
  public static final Climber climber = new Climber(); // My climber
  public static final AngleController angleController =
      new AngleController(); // My angle controller
  public static final LimelightShooter limelightShooter =
      new LimelightShooter(); // My limelight for the shooter
  public static final LimelightIntake limelightIntake =
      new LimelightIntake(); // My limelight for the intake
  public static final Slapper slapper = new Slapper(); // My slapper

  public static Drive drive = null;
}
