package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DrivetrainConstants;

public class Drive extends SubsystemBase {
    private SwerveModule m_frontLeft, m_frontRight;
    private SwerveModule m_backLeft, m_backRight;

    private final Translation2d m_frontLeftLocation, m_frontRightLocation;
    private final Translation2d m_backLeftLocation, m_backRightLocation;
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveDriveOdometry m_odometry;

    private final Gyro m_Gyro;

    public Drive(Gyro gyro) {
        m_Gyro = gyro;

        m_frontLeft = new SwerveModule(DrivetrainConstants.frontLeftDriveID, DrivetrainConstants.frontLeftSteerID, DrivetrainConstants.frontLeftCANCoderID, DrivetrainConstants.frontLeftEncoderOffset, 0);
        m_frontRight = new SwerveModule(DrivetrainConstants.frontRightDriveID, DrivetrainConstants.frontRightSteerID, DrivetrainConstants.frontRightCANCoderID, DrivetrainConstants.frontRightEncoderOffset, 1);
        m_backLeft = new SwerveModule(DrivetrainConstants.backLeftDriveID, DrivetrainConstants.backLeftSteerID, DrivetrainConstants.backLeftCANCoderID, DrivetrainConstants.backLeftEncoderOffset, 2);
        m_backRight = new SwerveModule(DrivetrainConstants.backRightDriveID, DrivetrainConstants.backRightSteerID, DrivetrainConstants.backRightCANCoderID, DrivetrainConstants.backRightEncoderOffset, 3);

        // TODO double check my negatives :^)
        m_frontLeftLocation = new Translation2d(-DrivetrainConstants.xOffsetMeters, DrivetrainConstants.yOffsetMeters);
        m_frontRightLocation = new Translation2d(DrivetrainConstants.xOffsetMeters, DrivetrainConstants.yOffsetMeters);
        m_backLeftLocation = new Translation2d(-DrivetrainConstants.xOffsetMeters, -DrivetrainConstants.yOffsetMeters);
        m_backRightLocation = new Translation2d(DrivetrainConstants.xOffsetMeters, -DrivetrainConstants.yOffsetMeters);

        m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        m_odometry = new SwerveDriveOdometry(m_kinematics, gyro.getGyroAngle(), getSwerveModulePositions());
    }

    /**
     * 
     * @param translation   linear movement (meters/sec)
     * @param rotation      rotational magnitude (radians/sec)
     * @param fieldOriented if true, swerve with respect to the bot
     */
    public void swerve(Translation2d translation, Double rotation, boolean fieldOriented) {
        System.out.println("Heading: " + m_Gyro.getGyroAngleClamped());
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                new Rotation2d(m_Gyro.getGyroAngleClamped())
            );
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        SwerveModuleState frontLeftState  = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(m_frontLeft.getAngle()));
        SwerveModuleState frontRightState = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(m_frontRight.getAngle()));
        SwerveModuleState backLeftState   = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(m_backLeft.getAngle()));
        SwerveModuleState backRightState  = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(m_backRight.getAngle()));

        m_frontLeft.setDesiredState(frontLeftState);
        m_frontRight.setDesiredState(frontRightState);
        m_backLeft.setDesiredState(backLeftState);
        m_backRight.setDesiredState(backRightState);
    }

    /**
     * swerve with respect to the field
     * @param translation   linear movement (meters/sec)
     * @param rotation      rotation (radians/sec)
     */
    public void swerve(Translation2d translation, Double rotation) {
        swerve(translation, rotation, true);
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = m_frontLeft.getPosition();
        positions[1] = m_frontRight.getPosition();
        positions[2] = m_backLeft.getPosition();
        positions[3] = m_backRight.getPosition();
        return positions;
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_Gyro.getGyroAngle(), getSwerveModulePositions(), pose);
    }

    @Override
    public void periodic() {
        m_frontLeft.updateSteer();
        m_frontRight.updateSteer();
        m_backLeft.updateSteer();
        m_backRight.updateSteer();
        m_odometry.update(m_Gyro.getGyroAngle(), getSwerveModulePositions());
    }

    public void goToAngle(double ang) {
        System.out.println("Passed angle: " + ang);

        SwerveModuleState frontLeftState = new SwerveModuleState(0d, new Rotation2d(Units.degreesToRadians(ang)));
        SwerveModuleState frontRightState = new SwerveModuleState(0d, new Rotation2d(Units.degreesToRadians(ang)));
        SwerveModuleState backLeftState = new SwerveModuleState(0d, new Rotation2d(Units.degreesToRadians(ang)));
        SwerveModuleState backRightState = new SwerveModuleState(0d, new Rotation2d(Units.degreesToRadians(ang)));

        m_frontLeft.setDesiredState(frontLeftState);
        m_frontRight.setDesiredState(frontRightState);
        m_backLeft.setDesiredState(backLeftState);
        m_backRight.setDesiredState(backRightState);
    }
}
