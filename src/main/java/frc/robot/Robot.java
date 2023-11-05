package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


public class Robot extends TimedRobot {

  private final XboxController m_controller = new XboxController(0);
	private final DriveTrain m_swerve = new DriveTrain();

  // Pigeon docs
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html#pigeon

  // Shufffleboard
  // https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-tour.html#tour-of-shuffleboard

  // Falcon setup example
  // private final WPI_TalonFX m_leftDrive = new WPI_TalonFX(1);
  // private final WPI_TalonFX m_rightDrive = new WPI_TalonFX(2);

  // Kinematics & Chassis Speeds class
  // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html

  // Swerve Drive Kinematics & Odometry
  // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
  // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html

  // Example docs for the majority of items
  // https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/wpilib-examples.html

  // Vision for when we get a POC
  // https://docs.wpilib.org/en/stable/docs/software/vision-processing/index.html#

  public void robotInit() {

  }

  public void robotPeriodic() {
  }

  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void disabledExit() {}

  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative){
    final var xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), 0.02) * DriveTrain.kMaxSpeed;
    final var ySpeed = -MathUtil.applyDeadband(m_controller.getLeftX(), 0.02) * DriveTrain.kMaxSpeed;
    final var rot = -MathUtil.applyDeadband(m_controller.getRightX(), 0.02) * DriveTrain.kMaxSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}

