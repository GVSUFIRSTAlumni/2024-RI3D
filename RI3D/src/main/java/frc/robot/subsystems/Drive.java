package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DrivetrainConstants;

public class Drive extends SubsystemBase {
    private final CANSparkMax m_frontLeftDrive,  m_frontLeftSteer,
                              m_frontRightDrive, m_frontRightSteer,
                              m_backLeftDrive,   m_backLeftSteer,
                              m_backRightDrive,  m_backRightSteer;

    private final SparkPIDController m_frontLeftDrivePID, m_frontLeftSteerPID,
                                     m_frontRightDrivePID, m_frontRightSteerPID,
                                     m_backLeftDrivePID, m_backLeftSteerPID,
                                     m_backRightDrivePID, m_backRightSteerPID;

    private final Translation2d m_frontLeftLocation, m_frontRightLocation;
    private final Translation2d m_backLeftLocation, m_backRightLocation;
    private final SwerveDriveKinematics m_kinematics;

    private final Gyro m_Gyro;

    public Drive(Gyro gyro) {
        m_Gyro = gyro;

        m_frontLeftDrive = new CANSparkMax(DrivetrainConstants.frontLeftDriveID, MotorType.kBrushless);
        m_frontLeftDrivePID = m_frontLeftDrive.getPIDController();
        m_frontLeftSteer = new CANSparkMax(DrivetrainConstants.frontLeftSteerID, MotorType.kBrushless);
        m_frontLeftSteerPID = m_frontLeftSteer.getPIDController();

        m_frontRightDrive = new CANSparkMax(DrivetrainConstants.frontRightDriveID, MotorType.kBrushless);
        m_frontRightDrivePID = m_frontRightDrive.getPIDController();
        m_frontRightSteer = new CANSparkMax(DrivetrainConstants.frontRightSteerID, MotorType.kBrushless);
        m_frontRightSteerPID = m_frontRightSteer.getPIDController();

        m_backLeftDrive = new CANSparkMax(DrivetrainConstants.backLeftDriveID, MotorType.kBrushless);
        m_backLeftDrivePID = m_backLeftDrive.getPIDController();
        m_backLeftSteer = new CANSparkMax(DrivetrainConstants.backLeftSteerID, MotorType.kBrushless);
        m_backLeftSteerPID = m_backLeftSteer.getPIDController();

        m_backRightDrive = new CANSparkMax(DrivetrainConstants.backRightDriveID, MotorType.kBrushless);
        m_backRightDrivePID = m_backRightDrive.getPIDController();
        m_backRightSteer = new CANSparkMax(DrivetrainConstants.backRightSteerID, MotorType.kBrushless);
        m_backRightSteerPID = m_backRightSteer.getPIDController();

        SparkPIDController[] driveControllers = {
            m_frontLeftDrivePID, m_frontRightDrivePID,
            m_backLeftDrivePID, m_backRightDrivePID
        };

        SparkPIDController[] steerControllers = {
            m_frontLeftSteerPID, m_frontRightSteerPID,
            m_backLeftSteerPID, m_backRightSteerPID
        };

        // configure drive PIDs
        for (var curController : driveControllers) {
            curController.setP(0d);
            curController.setI(0d);
            curController.setIZone(0d);
            curController.setD(0d);
            curController.setFF(0d);
            curController.setOutputRange(0d, 0d);
        }

        // configure steer PIDs
        for (var curController : steerControllers) {
            curController.setP(0d);
            curController.setI(0d);
            curController.setIZone(0d);
            curController.setD(0d);
            curController.setFF(0d);
            curController.setOutputRange(0d, 0d);
        }

        // values seem to be in meters - may want to confirm
        m_frontLeftLocation = new Translation2d(DrivetrainConstants.frontLeftXOffset, DrivetrainConstants.frontLeftYOffset);
        m_frontRightLocation = new Translation2d(DrivetrainConstants.frontRightXOffset, DrivetrainConstants.frontRightYOffset);
        m_backLeftLocation = new Translation2d(DrivetrainConstants.backLeftXOffset, DrivetrainConstants.backLeftYOffset);
        m_backRightLocation = new Translation2d(DrivetrainConstants.backRightXOffset, DrivetrainConstants.backRightYOffset);

        m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    }

    /**
     * 
     * @param forwardMag    forward/backward speed in m/s; +forward, -backward
     * @param sideMag       sideways speed in m/s; +left, -right
     * @param rotMag        rotation in radians/second
     */
    public void swerve(double linMag, double linAng, double rotMag) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            linMag,
            linAng,
            rotMag,
            new Rotation2d(m_Gyro.getGyroAngleClamped())
        );

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        // TODO rotation2d arg should be steer motor angle
        SwerveModuleState frontLeft  = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(0d));
        SwerveModuleState frontRight = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(0d));
        SwerveModuleState backLeft   = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(0d));
        SwerveModuleState backRight  = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(0d));

        // TODO pull values out of the SwerveModuleState objects and feed them to motors
    }

    @Override
    public void periodic() {}
}
