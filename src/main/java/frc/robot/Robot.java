package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

  private xbox = new XboxController(0);
  private final Timer m_timer = new Timer();
  Translation2d m_frontLeftLocation = new Translation2d(0.762, 0.762);
  Translation2d m_frontRightLocation = new Translation2d(0.762, -0.762);
  Translation2d m_backLeftLocation = new Translation2d(-0.762, 0.762);
  Translation2d m_backRightLocation = new Translation2d(-0.762, -0.762);

  SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  


  public void robotInit() {

  }

  public void robotPeriodic() {
  }

  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void disabledExit() {}

  public void autonomousInit() {

  }

  public void autonomousPeriodic() {}

  public void autonomousExit() {}

  public void teleopInit() {

  }

  public void teleopPeriodic() {}

  public void teleopExit() {}

  public void testInit() {
  }

  public void testPeriodic() {}

  public void testExit() {}
}
