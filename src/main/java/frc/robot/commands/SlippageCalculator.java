// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SlippageCalculator extends Command {
  private static final double linearSpeed = 1;
  // private static final double angularSpeed = 1;

  private SlipData data;

  private final Drive drivetrain;

  private Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization command. */
  public SlippageCalculator(Drive drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    data = new SlipData();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.acceptSimpleInput(linearSpeed, 0, 0, false);
    SwerveModuleState[] moduleState = drivetrain.getModuleStates();
    for (int i = 0; i < moduleState.length; i++) {
      Logger.recordOutput(
          "Slippage/WheelVelocity" + i, Math.abs(moduleState[i].speedMetersPerSecond));
    }
    data.add(moduleState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.acceptSimpleInput(0, 0, 0, false);
    timer.stop();
    data.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1.0;
  }

  public static class SlipData {
    private final List<SwerveModuleState[]> moduleStates = new LinkedList<>();

    public void add(SwerveModuleState[] moduleState) {
      moduleStates.add(moduleState);
    }

    public void print() {
      if (moduleStates.size() == 0) {
        return;
      }

      SwerveModuleState[][] stateData = moduleStates.toArray(new SwerveModuleState[0][]);

      double differenceAverage = 0;
      for (int i = 0; i < stateData.length; i++) {
        double[] moduleSpeeds = new double[4];
        for (int j = 0; j < stateData[i].length; j++) {
          moduleSpeeds[j] = Math.abs(stateData[i][j].speedMetersPerSecond);
        }
        double max = Arrays.stream(moduleSpeeds).max().getAsDouble();
        double average = Arrays.stream(moduleSpeeds).sum();
        average -= max;
        average /= 3;
        differenceAverage += max - average;
      }
      differenceAverage /= stateData.length;

      System.out.println("Slippage Calculator Results:");
      System.out.println("\tCount=" + stateData.length);
      System.out.println("\tAverage Difference=" + differenceAverage);
    }
  }
}
