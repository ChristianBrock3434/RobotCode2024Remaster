package frc.robot;

import static frc.robot.Constants.driver;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class Controls {
  private static final boolean rightStickDrive = true;
  private static final boolean dPadOreinted = true;

  // Drivetrain
  public static DoubleSupplier driveX =
      () -> rightStickDrive ? -driver.getRightY() : -driver.getLeftY();
  public static DoubleSupplier driveY =
      () -> rightStickDrive ? -driver.getRightX() : -driver.getLeftX();
  public static DoubleSupplier driveOmega =
      () -> rightStickDrive ? -driver.getLeftX() : -driver.getRightX();
  public static Trigger resetPose = driver.start();

  // Intake
  public static Trigger runIntake = driver.rightBumper();
  public static Trigger manualIn = dPadOreinted ? driver.x() : driver.pov(270);
  public static Trigger manualOut = dPadOreinted ? driver.b() : driver.pov(90);
  ;

  // Climber
  public static Trigger climberUp = dPadOreinted ? driver.pov(0) : driver.y();
  public static Trigger climberDown = dPadOreinted ? driver.pov(180) : driver.a();

  // Shooting
  public static Trigger subwooferShot = driver.rightTrigger(0.1);
  public static Trigger podiumShot = driver.leftTrigger(0.1);
  public static Trigger passShot = driver.leftBumper();
  public static Trigger ampShot = dPadOreinted ? driver.pov(270) : driver.x();

  public static Trigger cancelShot = dPadOreinted ? driver.pov(90) : driver.b();
}
