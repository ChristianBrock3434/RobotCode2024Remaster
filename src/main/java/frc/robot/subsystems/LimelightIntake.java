// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The `LimelightIntake` class represents a subsystem that controls the Limelight camera and its
 * functionalities. It provides methods to turn on/off the camera, set the light mode, switch the
 * pipeline, and retrieve information from the Limelight.
 */
public class LimelightIntake extends SubsystemBase {
  public final String LIMELIGHT = "limelight-intake";
  NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT);

  public static enum LightMode {
    DEFAULT(0),
    OFF(1),
    BLINK(2),
    ON(3);

    public int lightNum;

    private LightMode(int lightNum) {
      this.lightNum = lightNum;
    }
  };

  public static enum Pipeline {
    Note(0);
    // AprilTag3D(1);

    public int pipelineNum;

    private Pipeline(int pipelineNum) {
      this.pipelineNum = pipelineNum;
    }
  }

  /** Limelight Subsystem */
  public LimelightIntake() {
    turnOnLimelight();
  }

  /** Switches the camera to active and turns on the lights */
  public void turnOnLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(0);
    setLights(LightMode.DEFAULT);
  }

  /** Switches the camera to inactive and turns off the lights */
  public void turnOffLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(1);
    setLights(LightMode.OFF);
  }

  /**
   * Set the Light mode of the limelight
   *
   * @param lightMode LightMode enum value
   * @return Command to be scheduled
   */
  public Command setLightsCommand(LightMode lightMode) {
    return new Command() {
      @Override
      public void initialize() {
        setLights(lightMode);
      }
    };
  }

  /**
   * Sets light mode of the limelight
   *
   * @param lightMode LightMode enum value
   */
  public void setLights(LightMode lightMode) {
    NetworkTableInstance.getDefault()
        .getTable(LIMELIGHT)
        .getEntry("ledMode")
        .setNumber(lightMode.lightNum);
  }

  /**
   * Switches what the limelight is detecting
   *
   * @param pipeline Pipeline enum value
   * @return Command to be scheduled
   */
  public Command setLimelightPipelineCommand(Pipeline pipeline) {
    return new Command() {
      @Override
      public void initialize() {
        setLimelightPipeline(pipeline);
      }
    };
  }

  /**
   * Switches what the limelight is detecting
   *
   * @param pipeline Pipeline enum value
   */
  public void setLimelightPipeline(Pipeline pipeline) {
    NetworkTableInstance.getDefault()
        .getTable(LIMELIGHT)
        .getEntry("pipeline")
        .setNumber(pipeline.pipelineNum);
  }

  public Command prepareForNote() {
    return new Command() {
      @Override
      public void initialize() {
        turnOnLimelight();
        setLimelightPipeline(LimelightIntake.Pipeline.Note);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Scan Apriltag if you're in the right pipeline
   *
   * @return Id of Apriltag
   */
  public double getApriltagID() {
    double id = NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("tid").getDouble(0);
    // SmartDashboard.putNumber("Apriltag id", id);
    return id;
    // return 1;
  }

  /**
   * Prints the name of piece that the Limelight detects
   *
   * @param pipeline Pipeline enum value of the piece to detect
   * @return Command to be scheduled
   */
  public Command printPieceNameCommand(Pipeline pipeline) {
    return new Command() {
      @Override
      public void initialize() {
        turnOnLimelight();
        setLimelightPipeline(pipeline);
      }

      @Override
      public void execute() {
        printPieceName();
      }

      @Override
      public void end(boolean interrupted) {
        turnOffLimelight();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  /** Prints the name of piece that the Limelight detects */
  public void printPieceName() {
    System.out.println(
        NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("tclass").getString(null));
  }

  /**
   * Finds the pose of the robot when detecting 3D april tags
   *
   * @return An array of doubles in the order of X, Y, Z, Roll, Pitch, Yaw
   */
  public double[] getRobotPose() {
    double[] pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    try {
      pose =
          NetworkTableInstance.getDefault()
              .getTable(LIMELIGHT)
              .getEntry("botpose")
              .getDoubleArray(pose);
      // SmartDashboard.putNumber("PoseX", pose[0]);
      // SmartDashboard.putNumber("PoseY", pose[1]);
      // SmartDashboard.putNumber("PoseZ", pose[2]);
      // SmartDashboard.putNumber("PosePitch", pose[3]);
      // SmartDashboard.putNumber("PoseYaw", pose[4]);
      // SmartDashboard.putNumber("PoseRoll", pose[5]);
    } catch (ArrayIndexOutOfBoundsException e) {
      // System.out.println("No 3D April tag detected");
    }
    return pose;
    // return null;
  }

  /**
   * @return X position of the object (degrees)
   */
  public Double getTX() {
    double tX = table.getEntry("tx").getDouble(0);
    return (tX != 0) ? tX : Double.NaN;
    // return 1;
  }

  /**
   * @return Y position of the object (degrees)
   */
  public Double getTY() {
    double tY = table.getEntry("ty").getDouble(0);
    return (tY != 0) ? tY : Double.NaN;
    // return 1;
  }

  /**
   * @return Area of the screen the object takes up
   */
  public Double getTA() {
    double tA = table.getEntry("ta").getDouble(0);
    return (tA != 0) ? tA : Double.NaN;
    // return 1;
  }

  /**
   * @return Skew (rotation) of the object
   */
  public Double getTS() {
    double tS = table.getEntry("ts").getDouble(0);
    return (tS != 0) ? tS : Double.NaN;
    // return 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double[] pose = getRobotPose();
    // System.out.println("X: " + pose[0]);
    // System.out.println("Y: " + pose[1]);
    // System.out.println("Distance: " + getDistanceFromGoal());
    // System.out.println("TX: " + getTX());
  }
}
