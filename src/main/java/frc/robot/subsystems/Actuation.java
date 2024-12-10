// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ActuationConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The Actuation subsystem controls the actuation motor that moves a mechanism to a specific
 * position.
 */
public class Actuation extends SubsystemBase {
  private TalonFX actuationMotor = new TalonFX(14);

  private DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(6);

  private MotionMagicVoltage motionMagicControl;
  private VoltageOut voltageOut;
  private NeutralOut stopMode;

  private boolean isPositionControl;
  private double desiredPos;

  private PIDController posPID = new PIDController(1.5, 0, 0.01);
  // private PIDController paddingPID = new PIDController(1.5, 0, 0.0);

  /** Creates a new Actuation. */
  public Actuation() {
    initActuationMotor();

    motionMagicControl = new MotionMagicVoltage(0, true, -0.3, 0, false, false, false);

    voltageOut = new VoltageOut(0, true, false, false, false);

    desiredPos = actuationTuckPosition;
    posPID.disableContinuousInput();
    // posPID.setTolerance(1.5);

    stopMode = new NeutralOut();
  }

  /** Initialize the actuation motor */
  public void initActuationMotor() {
    // actuationMotor.setNeutralMode(NeutralModeValue.Brake);

    resetPosition();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // configs.MotorOutput.DutyCycleNeutralDeadband = 0.05;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 40;

    configs.MotionMagic.MotionMagicCruiseVelocity = 30;
    configs.MotionMagic.MotionMagicAcceleration = 50;
    configs.MotionMagic.MotionMagicJerk = 75;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 20; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.1; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = actuationMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Run the actuation motor to a given position
   *
   * @param position in encoder value
   * @return a command that will run the actuation motor
   */
  public Command setPositionCommand(double position) {
    return new Command() {
      @Override
      public void execute() {
        setPosition(position);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the actuation motor to a given position Slower but more stably
   *
   * @param position in encoder value
   * @return a command that will run the actuation motor
   */
  public Command setPositionStableCommand(double position) {
    return new Command() {
      @Override
      public void initialize() {
        // resetPosition();
      }

      @Override
      public void execute() {
        isPositionControl = false;
        actuationMotor.setControl(
            motionMagicControl.withPosition(position * actuationInternalTicksPerDegree));
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the actuation motor to a given position
   *
   * @param position in encoder value
   */
  public void setPosition(double position) {
    // System.out.println("Actuator Up");

    isPositionControl = true;
    desiredPos = position;

    // resetPosition();
  }

  /**
   * Run the actuation motor at a given percent
   *
   * @param speed 1 to -1
   */
  public Command runActuatorPercent(double speed) {
    return new Command() {
      @Override
      public void execute() {
        isPositionControl = false;
        actuationMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        actuationMotor.set(0);
      }
    };
  }

  /**
   * Reset the encoder to it's starting position
   *
   * @return a command that will reset the encoder
   */
  public Command resetPositionCommand() {
    return new Command() {
      @Override
      public void execute() {
        resetPosition();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * wait until the actuation motor is at a given position
   *
   * @param setPosition in degrees
   * @return a command that will wait until the actuation motor is at a given position
   */
  public Command waitUntilAtPosition(double setPosition) {
    return new Command() {
      @Override
      public boolean isFinished() {
        double currentPosition = getAngle();
        return Math.abs(
                currentPosition * actuationInternalTicksPerDegree
                    - setPosition * actuationInternalTicksPerDegree)
            <= 0.5;
      }
    };
  }

  private void runMotorToPosition() {
    if (desiredPos < 0) {
      posPID.setTolerance(1);
      // paddingPID.setTolerance(0.75);
    } else {
      posPID.setTolerance(2.5);
      // paddingPID.setTolerance(1.0);
    }

    double power =
        posPID.calculate(
            getAngle() * actuationInternalTicksPerDegree,
            desiredPos * actuationInternalTicksPerDegree);
    power = MathUtil.clamp(power, -3, 2.5);
    if (posPID.atSetpoint()) {
      actuationMotor.setControl(
          motionMagicControl.withPosition(desiredPos * actuationInternalTicksPerDegree));
    } else {
      actuationMotor.setControl(voltageOut.withOutput(power));
    }
  }

  /** Reset the actuation motor encoder to it's starting position */
  public void resetPosition() {
    // actuationMotor.setPosition(actuationStartPosition * actuationTicksPerDegree);
    double angle =
        throughboreEncoder.getAbsolutePosition() / actuationTicksPerDegree - actuationOffset;
    actuationMotor.setPosition(angle * actuationInternalTicksPerDegree);
  }

  /**
   * Get the actuation motor's current position
   *
   * @return the actuation motor's current position in degrees
   */
  public double getAngle() {
    // return throughboreEncoder.getAbsolutePosition() / actuationTicksPerDegree - actuationOffset;
    return actuationMotor.getPosition().getValueAsDouble() / actuationInternalTicksPerDegree;
    // return actuationMotor.getPosition().getValueAsDouble() / actuationTicksPerDegree;
  }

  public void stopMotor() {
    isPositionControl = false;
    actuationMotor.setControl(stopMode);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Actuation/EncoderAngle",
        (throughboreEncoder.getAbsolutePosition() / actuationTicksPerDegree) - actuationOffset);
    Logger.recordOutput(
        "Actuation/MotorAngle",
        actuationMotor.getPosition().getValueAsDouble() / actuationInternalTicksPerDegree);

    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("isPosControl", isPositionControl);
    if (isPositionControl) {
      runMotorToPosition();
    }

    SmartDashboard.putNumber(
        "actuation pos",
        (throughboreEncoder.getAbsolutePosition() / actuationTicksPerDegree) - actuationOffset);
    // System.out.println(getLimitSwitch());
    // System.out.println(actuationMotor.getPosition().getValueAsDouble() /
    // actuationTicksPerDegree);
    // System.out.println(actuationMotor.getMotorVoltage());

    SmartDashboard.putNumber(
        "Actuation Angle",
        actuationMotor.getPosition().getValueAsDouble() / actuationInternalTicksPerDegree);
  }
}
