package frc.robot.commands.drivetrain;

import static frc.robot.Subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;

public class ResetDrive extends Command {

  public ResetDrive() {
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.clearAutoAlignGoal();
    drive.clearHeadingGoal();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
