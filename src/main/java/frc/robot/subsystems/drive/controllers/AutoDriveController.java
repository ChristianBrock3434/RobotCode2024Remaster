// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Drive controller for outputting {@link ChassisSpeeds} from driver joysticks. */
public class AutoDriveController {

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  /**
   * Accepts new drive input from joysticks.
   *
   * @param x Desired x velocity scalar, -1..1
   * @param y Desired y velocity scalar, -1..1
   * @param omega Desired omega velocity scalar, -1..1
   * @param robotRelative Robot relative drive
   */
  public void acceptDriveInput(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  /**
   * Updates the controller with the currently stored state.
   *
   * @return {@link ChassisSpeeds} with driver's requested speeds.
   */
  public ChassisSpeeds update() {
    return chassisSpeeds;
  }
}
