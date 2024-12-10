package frc.robot.commands.automation;

import static frc.robot.Constants.AngleControllerConstants.angleRestingPosition;
import static frc.robot.Constants.SlapperConstants.slapperRestingPosition;
import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.angleController;
import static frc.robot.Subsystems.shooter;
import static frc.robot.Subsystems.slapper;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class represents a command group that zeros the angle of the shooter. It consists of a
 * sequence of commands that perform the necessary actions to zero the angle.
 */
public class ZeroAngle extends SequentialCommandGroup {

  public ZeroAngle() {
    addCommands(
        // new InstantCommand(actuation::stopMotor),
        new InstantCommand(shooter::stopMotors),
        slapper.setPositionCommand(slapperRestingPosition),
        new WaitCommand(0.5),
        new InstantCommand(angleController::runDown),
        angleController.waitUntilPressed().withTimeout(4),
        new InstantCommand(angleController::stopMotor),
        new InstantCommand(angleController::zeroOnSensor),
        actuation.resetPositionCommand(),
        new InstantCommand(slapper::resetPosition),
        // actuation.resetEncoderCommand(),
        angleController.setPositionCommand(angleRestingPosition),
        new InstantCommand(shooter::stopMotors));
  }

  public ZeroAngle(double angleRestingPosition) {
    addCommands(
        // new InstantCommand(actuation::stopMotor),
        new InstantCommand(shooter::stopMotors),
        slapper.setPositionCommand(slapperRestingPosition),
        new WaitCommand(0.25),
        new InstantCommand(angleController::runDown),
        angleController.waitUntilPressed().withTimeout(4),
        new InstantCommand(angleController::stopMotor),
        new InstantCommand(angleController::zeroOnSensor),
        actuation.resetPositionCommand(),
        new InstantCommand(slapper::resetPosition),
        // actuation.resetEncoderCommand(),
        angleController.setPositionCommand(angleRestingPosition),
        new InstantCommand(shooter::stopMotors));
  }
}
