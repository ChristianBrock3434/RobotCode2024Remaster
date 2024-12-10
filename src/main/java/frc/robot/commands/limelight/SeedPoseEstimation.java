package frc.robot.commands.limelight;

import static frc.robot.Subsystems.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class SeedPoseEstimation extends Command {
  double previousSeedTime = -10;

  public SeedPoseEstimation() {
    addRequirements(limelightShooter);
  }

  @Override
  public void initialize() {
    limelightShooter.turnOnLimelight();
  }

  @Override
  public void execute() {
    Rotation2d rot = RobotState.getInstance().getEstimatedPose().getRotation();

    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      DriverStation.reportWarning("ALLIANCE IS EMPTY, SELECT AN ALLIANCE", true);
    } else if (alliance.get().equals(Alliance.Red)) {
      rot = rot.plus(Rotation2d.fromDegrees(-180));
    }

    boolean rejectFrontLLUpdate = false;

    LimelightHelpers.SetRobotOrientation(
        limelightShooter.LIMELIGHT,
        rot.getDegrees(),
        RobotState.getInstance().fieldVelocity().dtheta,
        0,
        0,
        0,
        0);
    // LimelightHelpers.SetRobotOrientation(limelightShooter.LIMELIGHT, rot.getDegrees(), 0, 0, 0,
    // 0, 0);
    LimelightHelpers.PoseEstimate poseEstimate_FrontLL =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightShooter.LIMELIGHT);

    if (RobotState.getInstance().fieldVelocity().dtheta
        > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision
    // updates
    {
      rejectFrontLLUpdate = true;
    }

    Logger.recordOutput("Limelight/rotVel", RobotState.getInstance().fieldVelocity().dtheta);
    Logger.recordOutput("Limelight/Pose", poseEstimate_FrontLL.pose);

    try {
      if (!rejectFrontLLUpdate
          && !(poseEstimate_FrontLL.pose.getX() == 0)
          && !(poseEstimate_FrontLL.pose.getY() == 0)
          && poseEstimate_FrontLL.timestampSeconds - previousSeedTime > 0.25) {
        RobotState.getInstance()
            .addVisionObservation(
                new RobotState.VisionObservation(
                    poseEstimate_FrontLL.pose, poseEstimate_FrontLL.timestampSeconds));
        previousSeedTime = poseEstimate_FrontLL.timestampSeconds;
      }
    } catch (Exception e) {
      // System.out.println(e);
    }
  }

  @Override
  public void end(boolean isInterrupted) {
    // limelightShooter.turnOffLimelight();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
