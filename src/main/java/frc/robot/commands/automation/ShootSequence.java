package frc.robot.commands.automation;

import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShakeController;
import java.util.function.DoubleSupplier;

/**
 * A command that represents the sequence of actions required to perform a shooting operation. This
 * command includes setting the angle, speeding up the shooter, waiting until the angle is reached,
 * checking if the shooter is at the desired speed, speeding up the indexer, checking if the indexer
 * is at the desired speed, and feeding the intake.
 */
public class ShootSequence extends ConditionalCommand {

  /**
   * Constructs a new ShootSequence command.
   *
   * @param angle a supplier for the desired angle of the shooter
   * @param velocity a supplier for the desired velocity of the shooter
   */
  public ShootSequence(DoubleSupplier angle, DoubleSupplier velocity, DoubleSupplier slapperAngle) {
    super(
        new SequentialCommandGroup(
            angleController.setPositionCommandSupplier(angle),
            slapper.setPositionCommand(slapperAngle),
            shooter.speedUpShooter(velocity, shooterSequenceAcceleration),
            angleController.waitUntilAtPositionSupplier(angle),
            shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.8),
            indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
            shooter.checkIfAtSpeedSupplier(velocity),
            indexer.checkIfAtSpeedSupplier(() -> indexerVelocity),
            intake.feedCommand(feedVelocity, feedAcceleration)),
        new ShakeController(1.0, 1.0),
        () ->
            !(((Double) angle.getAsDouble()).equals(Double.NaN))
                || !(((Double) velocity.getAsDouble()).equals(Double.NaN)));
  }

  /**
   * Constructs a new ShootSequence command.
   *
   * @param angle a supplier for the desired angle of the shooter
   * @param velocity a supplier for the desired velocity of the shooter
   */
  public ShootSequence(
      DoubleSupplier angle,
      DoubleSupplier velocity,
      double restingAngle,
      DoubleSupplier slapperAngle,
      double slapperRestingPosition) {
    super(
        new SequentialCommandGroup(
            angleController.setPositionCommandSupplier(angle),
            slapper.setPositionCommand(slapperAngle),
            shooter.speedUpShooter(velocity, shooterSequenceAcceleration),
            angleController.waitUntilAtPositionSupplier(angle),
            shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.75),
            indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
            shooter.checkIfAtSpeedSupplier(velocity),
            indexer.checkIfAtSpeedSupplier(() -> indexerVelocity),
            actuation.waitUntilAtPosition(actuationTuckPosition),
            intake.startFeedingCommand(feedVelocity, feedAcceleration),
            new WaitCommand(1.0),
            new StopShoot(restingAngle, slapperRestingPosition)),
        new ShakeController(1.0, 1.0),
        () ->
            !(((Double) angle.getAsDouble()).equals(Double.NaN))
                || !(((Double) velocity.getAsDouble()).equals(Double.NaN)));
  }

  public ShootSequence(
      DoubleSupplier angle,
      DoubleSupplier velocity,
      double restingAngle,
      double indexerVelocity,
      DoubleSupplier slapperAngle,
      double slapperRestingPosition) {
    super(
        new SequentialCommandGroup(
            angleController.setPositionCommandSupplier(angle),
            slapper.setPositionCommand(slapperAngle),
            shooter.speedUpShooterSlow(velocity, shooterSequenceAcceleration),
            angleController.waitUntilAtPositionSupplier(angle),
            shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.75),
            indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
            shooter.checkIfAtSpeedSupplier(velocity),
            indexer.checkIfAtSpeedSupplier(() -> indexerVelocity),
            actuation.waitUntilAtPosition(actuationTuckPosition),
            intake.startFeedingCommand(feedVelocity, feedAcceleration),
            new WaitCommand(1.0),
            // shooter.waitUntilRingLeft(),
            new StopShoot(restingAngle, slapperRestingPosition)),
        new ShakeController(1.0, 1.0),
        () ->
            !(((Double) angle.getAsDouble()).equals(Double.NaN))
                || !(((Double) velocity.getAsDouble()).equals(Double.NaN)));
  }
}
