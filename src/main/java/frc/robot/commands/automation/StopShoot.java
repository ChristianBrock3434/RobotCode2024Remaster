package frc.robot.commands.automation;

import static frc.robot.Constants.SlapperConstants.slapperRestingPosition;
import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * A command group that stops the shooter, indexer motor, intake, and sets the angle of the shooter.
 */
public class StopShoot extends ParallelCommandGroup {

  /**
   * Constructs a new StopShoot command group with the specified angle.
   *
   * @param angle the angle to set the shooter to
   */
  public StopShoot(double angle, double slapperRestingPosition) {
    addCommands(
        new InstantCommand(shooter::stopMotors, shooter),
        new InstantCommand(indexer::stopIndexerMotor, indexer),
        intake.stopIntakeCommand(),
        angleController.setPositionCommand(angle),
        slapper.setPositionCommand(slapperRestingPosition));
  }
}
