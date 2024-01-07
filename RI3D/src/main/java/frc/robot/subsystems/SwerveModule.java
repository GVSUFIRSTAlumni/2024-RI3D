package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule {
    private final CANSparkMax m_driveMotor, m_steerMotor;
    private final SparkPIDController m_drivePID;
    private final PIDController m_steerPID;
    private final WPI_CANCoder m_CANCoder;
    private final Rotation2d m_encoderOffset;
    private Rotation2d m_lastAngle;
    private SwerveModuleState m_curState;

    /**
     * 
     * @param driveID       CAN ID of the drive motor
     * @param steerID       CAN ID of the steer motor
     * @param encoderID     CAN ID of the CANCoder
     * @param encoderOffset starting encoder position
     */
    public SwerveModule(final int driveID, final int steerID, final int encoderID, final double encoderOffset) {
        m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);       
        m_CANCoder = new WPI_CANCoder(encoderID);
        m_encoderOffset = new Rotation2d(encoderOffset);

        m_drivePID = m_driveMotor.getPIDController();
        m_steerPID = new PIDController(0.01, 0, 0); // TODO: set theses
        
        m_lastAngle = getState().angle;

        configDrive();
    }

    /**
     * 
     * @param state 
     */
    public void setDesiredState(SwerveModuleState state)  {
        // setSpeed(state);
        setAngle(state);
    }

    /**
     * 
     * @return
     */
    private SwerveModuleState getState() {
        if (m_curState == null) {
            m_curState = new SwerveModuleState();
        }

        m_curState.angle = new Rotation2d(getAngle());
        m_curState.speedMetersPerSecond = getSpeed();

        return m_curState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), new Rotation2d(getAngle()));
    }

    
    /**
     * Gets the speed of the drive motor.
     * @return the current speed in meters per second.
     */
    private double getSpeed() {
        return m_driveMotor.getEncoder().getVelocity();
    }

    /**
     * 
     * @param state
     */
    public void setSpeed(SwerveModuleState state) {
        m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVoltage);
    }

    /**
     * 
     * @return 
     */
    public double getAngle() {
        // 360 degrees in a circle divided by 4096 encoder counts/revolution (CANCoder resolution)
        return (m_CANCoder.getPosition() * (360 / 4096)) - m_encoderOffset.getDegrees();
    }

    /**
     * 
     * @param state
     */
    private void setAngle(SwerveModuleState state) {
        //Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (DrivetrainConstants.maxSpeed * 0.01)) ? m_lastAngle : state.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = state.angle;
        m_steerPID.setSetpoint(angle.getDegrees());
        m_lastAngle = angle;
    }

    private double getDistance() {
        return m_driveMotor.getEncoder().getPosition();
    }

    /**
     * 
     */
    private void configDrive() {
        m_driveMotor.restoreFactoryDefaults();

        m_driveMotor.getEncoder().setVelocityConversionFactor((Units.inchesToMeters(3) * Math.PI / 6.75)); // in meters per second
        m_driveMotor.getEncoder().setPositionConversionFactor((Units.inchesToMeters(3) * Math.PI / 6.75));

        m_drivePID.setP(DrivetrainConstants.DriveParams.kP);
        m_drivePID.setI(DrivetrainConstants.DriveParams.kI);
        m_drivePID.setD(DrivetrainConstants.DriveParams.kD);
        m_drivePID.setFF(DrivetrainConstants.DriveParams.kFF);

        m_driveMotor.setIdleMode(DrivetrainConstants.DriveParams.kIdleMode);

        m_driveMotor.burnFlash();
    }

    public void updateSteer() {
        m_steerMotor.setVoltage(m_steerPID.calculate(getAngle()));
    }
}
