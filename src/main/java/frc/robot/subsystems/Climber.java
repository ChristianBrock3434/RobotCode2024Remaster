package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The `Climber` class represents a subsystem that controls the climber mechanism of the robot. It
 * provides methods to control the climber motor, set the position of the climber, and run the
 * climber at a specified voltage.
 */
public class Climber extends SubsystemBase {
  private TalonFX climberMotor = new TalonFX(18);

  private VoltageOut voltageControl;
  private MotionMagicVoltage motionMagicControl;
  private VelocityVoltage velocityControl;
  private NeutralOut StopMode;

  public Climber() {
    initClimberMotor();

    voltageControl = new VoltageOut(0.0, true, false, false, false);

    motionMagicControl = new MotionMagicVoltage(0, true, 0.03, 0, false, false, false);

    // only used for tuning feed forward
    velocityControl = new VelocityVoltage(100, 100, true, 0.03, 0, false, false, false);

    StopMode = new NeutralOut();
  }

  /** Initialize the climber motor */
  public void initClimberMotor() {
    resetClimber();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    configs.MotionMagic.MotionMagicCruiseVelocity = 130;
    configs.MotionMagic.MotionMagicAcceleration = 250;
    configs.MotionMagic.MotionMagicJerk = 350;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 10; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = climberMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Creates a command that controls the climber motor at a specified percentage.
   *
   * @param percent The percentage at which to control the climber motor.
   * @return The command object that controls the climber motor.
   */
  public Command climberPercentControl(double percent) {
    return new Command() {
      @Override
      public void execute() {
        climberMotor.set(percent);
      }

      @Override
      public void end(boolean interrupted) {
        climberMotor.set(0);
      }
    };
  }

  /**
   * Run the Climber motor at a given voltage, Stopping at it's max height and lowest height
   *
   * @param voltage in volts
   * @return a command that will run the intake motor
   */
  public Command runLimitedVoltageCommand(double voltage) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Climber.this);
      }

      @Override
      public void execute() {
        if (climberMotor.getPosition().getValueAsDouble() < minClimberHeight && voltage < 0) {
          stopClimberMotor();
        } else if (climberMotor.getPosition().getValueAsDouble() > maxClimberHeight
            && voltage > 0) {
          stopClimberMotor();
        } else {
          runVoltage(voltage);
        }
      }

      @Override
      public void end(boolean interrupted) {
        stopClimberMotor();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  /**
   * Run the climber motor to a given position
   *
   * @param position in encoder value
   * @return a command that will run the climber motor
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
   * Run the climber motor to a given position
   *
   * @param position in encoder value
   */
  public void setPosition(double position) {
    climberMotor.setControl(motionMagicControl.withPosition(position));
  }

  /**
   * Run the Climber motor at a given voltage
   *
   * @param voltage in volts
   * @return a command that will run the intake motor
   */
  public Command runVoltageCommand(double voltage) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Climber.this);
      }

      @Override
      public void execute() {
        runVoltage(voltage);
      }

      @Override
      public void end(boolean interrupted) {
        stopClimberMotor();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  /**
   * Run the Climber motor at a given voltage
   *
   * @param voltage in volts
   */
  public void runVoltage(double voltage) {
    climberMotor.setControl(voltageControl.withOutput(voltage));
  }

  /**
   * Runs the climber motor at a specified velocity.
   *
   * @return The command to run the climber motor at a specified velocity.
   */
  public Command runVelocity() {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Climber.this);
      }

      @Override
      public void execute() {
        climberMotor.setControl(velocityControl);
      }

      @Override
      public void end(boolean interrupted) {
        stopClimberMotor();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  /** Stop the Climber motor */
  public void stopClimberMotor() {
    climberMotor.setControl(StopMode);
  }

  /** Reset the Climber encoder */
  public void resetClimber() {
    climberMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("Climber Position: " + climberMotor.getPosition().getValueAsDouble());
    // System.out.println("Climber velocity: " + climberMotor.getVelocity().getValueAsDouble());
    // System.out.println("Climber voltage: " + climberMotor.getMotorVoltage().getValueAsDouble());
  }
}
