package frc.robot.commands.automation;

import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.AngleControllerConstants.angleRestingPosition;
import static frc.robot.Constants.AngleControllerConstants.trapAngle;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.SlapperConstants.slapperRestingPosition;
// import static frc.robot.Constants.SlapperConstants.slapperTrapPosition;
import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** A command group that represents the sequence of commands to shoot at the trap. */
public class ShootTrap extends SequentialCommandGroup {

  public ShootTrap() {
    addCommands(
        // new PrintCommand("Angle: " + angle.getAsDouble()),
        // new PrintCommand("Speed: " + velocity.getAsDouble())
        angleController.setPositionCommandSupplier(() -> trapAngle),
        slapper.setPositionCommand(slapperRestingPosition),
        shooter.speedUpShooter(() -> trapSpeed, shooterSequenceAcceleration),
        angleController.waitUntilAtPositionSupplier(() -> trapAngle),
        shooter.checkIfAtSpeedSupplier(() -> trapSpeed * 0.8),
        indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
        shooter.checkIfAtSpeedSupplier(() -> trapSpeed),
        indexer.checkIfAtSpeedSupplier(() -> indexerVelocity),
        actuation.waitUntilAtPosition(actuationTuckPosition),
        intake.startFeedingCommand(feedVelocity, feedAcceleration),
        new WaitCommand(0.5).raceWith(shooter.waitUntilRingLeft()),
        new StopShoot(angleRestingPosition, slapperRestingPosition));
  }
}
