package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveModule {
    private static final double kWheelRadius = 0.0002; //CHANGE ME!

    private static final double kModuleMaxAngularVelocity = DriveTrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; 

    private final MotorController m_driveMotor;
    private final MotorController m_turningMotor;

    // Encoders CTRE Mag Encoder
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#duty-cycle-encoders-the-dutycycleencoder-class

    public SwerveModule(int driveMotorChannel, int turningMotorChannel){
        
    }
}
