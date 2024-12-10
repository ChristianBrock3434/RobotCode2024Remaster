package frc.robot.commands.automation;

import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShakeController;

/**
 * A command group that represents the process of picking up a piece. This command group
 * sequentially executes a series of commands to pick up a piece, including positioning the
 * actuation mechanism, running the intake, and tucking the actuation mechanism. It also includes a
 * shake controller to add a shaking motion after picking up the piece.
 *
 * @param voltage The voltage to run the intake at.
 */
public class PickUpPiece extends SequentialCommandGroup {

  public PickUpPiece(double voltage) {
    addCommands(
        actuation.setPositionCommand(actuationPickUpPosition),
        actuation.waitUntilAtPosition(actuationPickUpPosition),
        intake.runVoltageCommand(voltage),
        intake.waitUntilTripped(),
        intake.stopIntakeCommand(),
        actuation.setPositionCommand(actuationTuckPosition),
        new ShakeController(1.0, 1.0));
  }
}
