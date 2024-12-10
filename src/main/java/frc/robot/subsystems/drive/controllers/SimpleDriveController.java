// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.controllers;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;

/** Drive controller for outputting {@link ChassisSpeeds} from driver joysticks. */
public class SimpleDriveController {
  private double X = 0;
  private double Y = 0;
  private double omega = 0;
  private boolean robotRelative = false;

  public void acceptDriveInput(double x, double y, double omega, boolean robotRelative) {
    this.X = x;
    this.Y = y;
    this.omega = omega;
    this.robotRelative = robotRelative;
  }

  /**
   * Updates the controller with the currently stored state.
   *
   * @return {@link ChassisSpeeds} with driver's requested speeds.
   */
  public ChassisSpeeds update() {
    Translation2d linearVelocity = calcLinearVelocity(X, Y);
    omega = Math.copySign(omega * omega, omega);

    final double maxLinearVelocity = driveConfig.maxLinearVelocity();
    final double maxAngularVelocity = driveConfig.maxAngularVelocity();
    if (robotRelative) {
      return new ChassisSpeeds(
          linearVelocity.getX() * maxLinearVelocity,
          linearVelocity.getY() * maxLinearVelocity,
          omega * maxAngularVelocity);
    } else {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
      }
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          linearVelocity.getX() * maxLinearVelocity,
          linearVelocity.getY() * maxLinearVelocity,
          omega * maxAngularVelocity,
          RobotState.getInstance().getEstimatedPose().getRotation());
    }
  }

  public static Translation2d calcLinearVelocity(double x, double y) {
    // Apply deadband
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square magnitude
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    return linearVelocity;
  }
}
