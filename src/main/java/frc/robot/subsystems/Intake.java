// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Intake class represents the intake subsystem of the robot. It controls the intake motor and
 * provides methods to run the motor at different velocities and voltages.
 */
public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(13);

  private DigitalInput rightNoteSensor = new DigitalInput(0);
  private DigitalInput leftNoteSensor = new DigitalInput(1);

  VelocityVoltage velocityControlFeed;
  VoltageOut voltageControl;
  NeutralOut stopMode;

  /** Creates a new Intake. */
  public Intake() {
    initIntakeMotor();

    velocityControlFeed = new VelocityVoltage(0, 0, true, 0.5, 1, false, false, false);

    voltageControl = new VoltageOut(0, false, false, false, false);

    stopMode = new NeutralOut();
  }

  /** Initialize the intake motor */
  public void initIntakeMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 30;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    configs.Slot1.kP = 4; // An error of 1 rotation per second results in 2V output
    configs.Slot1.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot1.kD =
        0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot1.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   *
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @param timeout in seconds; use 0 for no timeout
   * @return a command that will run the intake motor
   */
  public Command feedCommand(double velocity, double acceleration) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Intake.this);
      }

      @Override
      public void execute() {
        feedMotor(velocity, acceleration);
      }

      @Override
      public void end(boolean interrupted) {
        stopIntakeMotor();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   *
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the intake motor
   */
  public Command startFeedingCommand(double velocity, double acceleration) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Intake.this);
        feedMotor(velocity, acceleration);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   *
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void feedMotor(double velocity, double acceleration) {
    intakeMotor.setControl(
        velocityControlFeed.withVelocity(velocity).withAcceleration(acceleration));
  }

  /**
   * Run the intake motor at a given voltage
   *
   * @param voltage in volts
   * @return a command that will run the intake motor
   */
  public Command runVoltageCommand(double voltage) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Intake.this);
      }

      @Override
      public void execute() {
        runVoltage(voltage);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the intake motor at a given voltage
   *
   * @param voltage in volts
   */
  public void runVoltage(double voltage) {
    intakeMotor.setControl(voltageControl.withOutput(voltage));
  }

  /**
   * Run the intake motor at a given percent
   *
   * @param speed 1 to -1
   * @return a command that will run the intake motor
   */
  public Command runIntakePercent(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Intake.this);
      }

      @Override
      public void execute() {
        intakeMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        intakeMotor.set(0);
      }
    };
  }

  /**
   * Stop the intake motor
   *
   * @return a command that will stop the intake motor
   */
  public Command stopIntakeCommand() {
    // return stopIntakeFast().withTimeout(0.2);
    return new InstantCommand(this::stopIntakeMotor, this);
  }

  /**
   * Stop the intake motor quickly
   *
   * @return a command that will stop the intake motor
   */
  @Deprecated
  public Command stopIntakeFast() {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Intake.this);
        intakeMotor.setControl(velocityControlFeed.withVelocity(0).withAcceleration(100));
      }

      @Override
      public void end(boolean interrupted) {
        stopIntakeMotor();
      }
    };
  }

  /** Stop the intake motor */
  public void stopIntakeMotor() {
    // System.out.println("Intake stopped");
    intakeMotor.setControl(stopMode);
    // intakeMotor.setControl(velocityControlFeed.withVelocity(0).withAcceleration(20));
  }

  /**
   * Wait until the note sensor is tripped
   *
   * @return a command that will wait until the note sensor is tripped
   */
  public Command waitUntilTripped() {
    return new Command() {
      @Override
      public void execute() {
        if (getNoteSensor()) {
          stopIntakeMotor();
        }
      }

      @Override
      public boolean isFinished() {
        return getNoteSensor();
      }
    };
  }

  /**
   * Wait until either note sensor is tripped
   *
   * @return a command that will wait until the note sensor is tripped
   */
  public boolean getNoteSensor() {
    return leftNoteSensor.get() || rightNoteSensor.get();
  }

  /**
   * Get the state of the left note sensor
   *
   * @return true if the sensor is tripped
   */
  public boolean getLeftNoteSensor() {
    return leftNoteSensor.get();
  }

  /**
   * Get the state of the right note sensor
   *
   * @return true if the sensor is tripped
   */
  public boolean getRightNoteSensor() {
    return rightNoteSensor.get();
  }

  /**
   * Get the velocity of the intake motor
   *
   * @return the velocity in rotations per second
   */
  public double getVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(intakeMotor.getDeviceTemp().toString());
    // System.out.println(pdp.getCurrent(16));
    // System.out.println(getNoteSensor());
    // if (getNoteSensor()) {
    //   System.out.println("detect");
    // }
    // SmartDashboard.putBoolean("Left Intake Note Sensor", getLeftNoteSensor());
    // SmartDashboard.putBoolean("Right Intake Note Sensor", getRightNoteSensor());

    // SmartDashboard.putNumber("Intake Speed", intakeMotor.getVelocity().getValueAsDouble());
  }
}
