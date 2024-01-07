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
    private SwerveModule m_frontLeft, m_frontRight;
    private SwerveModule m_backLeft, m_backRight;

    private final Translation2d m_frontLeftLocation, m_frontRightLocation;
    private final Translation2d m_backLeftLocation, m_backRightLocation;
    private final SwerveDriveKinematics m_kinematics;

    private final Gyro m_Gyro;

    public Drive(Gyro gyro) {
        m_Gyro = gyro;

        m_frontLeft = new SwerveModule(DrivetrainConstants.frontLeftDriveID, DrivetrainConstants.frontLeftSteerID, DrivetrainConstants.frontLeftCANCoderID, 0d);

        // TODO double check my negatives :^)
        m_frontLeftLocation = new Translation2d(-DrivetrainConstants.xOffsetMeters, DrivetrainConstants.yOffsetMeters);
        m_frontRightLocation = new Translation2d(DrivetrainConstants.xOffsetMeters, DrivetrainConstants.yOffsetMeters);
        m_backLeftLocation = new Translation2d(-DrivetrainConstants.xOffsetMeters, -DrivetrainConstants.yOffsetMeters);
        m_backRightLocation = new Translation2d(DrivetrainConstants.xOffsetMeters, -DrivetrainConstants.yOffsetMeters);

        m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    }

    /**
     * 
     * @param forwardMag    forward/backward speed in m/s; +forward, -backward
     * @param sideMag       sideways speed in m/s; +left, -right
     * @param rotMag        rotation in radians/second
     * @param fieldOriented if true, is field oriented
     */
    public void swerve(Translation2d translation, Rotation2d rotation, boolean fieldOriented) {
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians(),
                new Rotation2d(m_Gyro.getGyroAngleClamped())
            );
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation.getRadians());
        }

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        // TODO rotation2d arg should be steer motor angle
        SwerveModuleState frontLeftState  = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(0d));
        SwerveModuleState frontRightState = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(0d));
        SwerveModuleState backLeftState   = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(0d));
        SwerveModuleState backRightState  = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(0d));

        m_frontLeft.setDesiredState(frontLeftState);
        m_frontRight.setDesiredState(frontRightState);
        m_backLeft.setDesiredState(backLeftState);
        m_backRight.setDesiredState(backRightState);
    }

    /**
     * swerve with respect to the field
     * @param linMag linear magnitude - controls speed
     * @param linAng linear angle - contro
     * @param rotMag
     */
    public void swerve(double linMag, double linAng, double rotMag) {
        swerve(linMag, linAng, rotMag, true);
    }

    @Override
    public void periodic() {}
}
