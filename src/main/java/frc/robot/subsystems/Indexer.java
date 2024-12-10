// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/**
 * The Indexer class represents a subsystem that controls the indexer motor. It provides methods to
 * initialize, run, and stop the indexer motor, as well as check its speed and create commands to
 * control it.
 */
public class Indexer extends SubsystemBase {
  private TalonFX indexerMotor = new TalonFX(17);

  VelocityVoltage velocityControl;
  NeutralOut stopMode;

  /** Creates a new indexer. */
  public Indexer() {
    initIndexerMotor();

    velocityControl = new VelocityVoltage(0, 0, true, 0.6, 0, false, false, false);

    stopMode = new NeutralOut();
  }

  /** Initialize the indexer motor */
  public void initIndexerMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 30;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.6; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = indexerMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Run the indexer motor at a given velocity and acceleration
   *
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the indexer motor
   */
  public Command runIndexerCommand(double velocity, double acceleration) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Indexer.this);
      }

      @Override
      public void execute() {
        runIndexer(velocity, acceleration);
      }

      @Override
      public void end(boolean interrupted) {
        stopIndexerMotor();
      }
    };
  }

  /**
   * Run the indexer motor at a given velocity and acceleration
   *
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the indexer motor
   */
  public Command speedUpIndexer(double velocity, double acceleration) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Indexer.this);
        runIndexer(velocity, acceleration);
        // System.out.println("Indexer Speed:" + velocity);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the indexer motor at a given velocity and acceleration
   *
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void runIndexer(double velocity, double acceleration) {
    indexerMotor.setControl(velocityControl.withVelocity(velocity).withAcceleration(acceleration));
  }

  /**
   * Run the indexer motor at a given percentage speed
   *
   * @param speed 1 to -1
   * @return a command that will run the indexer motor
   */
  public Command runIndexerPercent(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Indexer.this);
      }

      @Override
      public void execute() {
        indexerMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        indexerMotor.set(0);
      }
    };
  }

  /** Stop the indexer motor */
  public void stopIndexerMotor() {
    indexerMotor.setControl(stopMode);
  }

  /**
   * Checks if the indexer is at a certain speed
   *
   * @param velocity the speed to check for in rotations per second
   * @return a command that will wait until the indexer is at a certain speed
   */
  public Command checkIfAtSpeedSupplier(DoubleSupplier velocity) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        if (((Double) velocity.getAsDouble()).equals(Double.NaN)) {
          return true;
        }
        return indexerMotor.getVelocity().getValueAsDouble() >= velocity.getAsDouble() * 0.90;
      }
    };
  }

  /**
   * Get the current speed of the indexer
   *
   * @return the speed of the indexer in rotations per second
   */
  public double getVelocity() {
    return indexerMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(pdp.getCurrent(16));
    // System.out.println(indexerMotor.getDeviceTemp().getValueAsDouble());
    // System.out.println("Indexer Speed: " + indexerMotor.getVelocity());
    // SmartDashboard.putNumber("Indexer Speed", indexerMotor.getVelocity().getValueAsDouble());
  }
}
