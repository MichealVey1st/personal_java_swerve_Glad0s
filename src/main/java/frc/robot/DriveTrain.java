package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;

public class DriveTrain{
    public static final double kMaxSpeed = 3.5;
    public static final double kMaxAngularSpeed = Math.PI;

    private final Translation2d m_frontLeftLocation = new Translation2d(0.762, 0.762);
    private final Translation2d m_frontRightLocation = new Translation2d(0.762, -0.762);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.762, 0.762);
    private final Translation2d m_backRightLocation = new Translation2d(-0.762, -0.762);

    private final SwerveModule m_frontLeft = new SwerveModule()
}
