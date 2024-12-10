package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.Subsystems.limelightIntake;
import static frc.robot.Subsystems.limelightShooter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;

/**
 * The ShakeController class represents a command that shakes the controller for a specified
 * duration and intensity.
 */
public class ShakeController extends Command {
  private double startingTime;
  private double rumbleIntensity;
  private double shakeTime;

  /**
   * Constructs a new ShakeController command with the specified rumble intensity and shake time.
   *
   * @param rumbleIntensity The intensity of the controller rumble.
   * @param shakeTime The duration of the controller shake in seconds.
   */
  public ShakeController(double rumbleIntensity, double shakeTime) {
    this.rumbleIntensity = rumbleIntensity;
    this.shakeTime = shakeTime;
  }

  @Override
  public void initialize() {
    startingTime = System.currentTimeMillis();
    driver.getHID().setRumble(RumbleType.kBothRumble, rumbleIntensity);
    limelightIntake.setLights(LimelightIntake.LightMode.BLINK);
    limelightShooter.setLights(LimelightShooter.LightMode.BLINK);
  }

  @Override
  public void end(boolean interrupted) {
    driver.getHID().setRumble(RumbleType.kBothRumble, 0);
    limelightIntake.setLights(LimelightIntake.LightMode.DEFAULT);
    limelightShooter.setLights(LimelightShooter.LightMode.DEFAULT);
  }

  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() - startingTime > shakeTime * 1000) {
      return true;
    }
    return false;
  }
}
