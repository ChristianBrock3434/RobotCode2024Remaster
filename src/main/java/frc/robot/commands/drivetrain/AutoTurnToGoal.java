package frc.robot.commands.drivetrain;

import static frc.robot.Subsystems.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.Speaker;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoTurnToGoal extends Command {

  protected Supplier<Rotation2d> angleSupplier;
  private DoubleSupplier offset;

  public AutoTurnToGoal(DoubleSupplier offset) {
    this.offset = offset;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Translation2d speakerLocation =
        AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d());
    this.angleSupplier =
        () -> {
          Transform2d translation =
              new Transform2d(
                  speakerLocation.getX() - RobotState.getInstance().getEstimatedPose().getX(),
                  speakerLocation.getY() - RobotState.getInstance().getEstimatedPose().getY(),
                  new Rotation2d());
          return new Rotation2d(
              Math.atan2(translation.getY(), translation.getX())
                  + Units.degreesToRadians(offset.getAsDouble()));
        };
    drive.setHeadingGoal(angleSupplier);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    drive.clearHeadingGoal();
  }

  @Override
  public boolean isFinished() {
    return drive.atHeadingGoal();
  }
}
