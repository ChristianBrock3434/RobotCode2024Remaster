package frc.robot.commands.automation;

import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** A command group that stops the intake and sets the actuation position to a tuck position. */
public class StopIntake extends ParallelCommandGroup {
  public StopIntake() {
    addCommands(intake.stopIntakeCommand(), actuation.setPositionCommand(actuationTuckPosition));
  }
}
