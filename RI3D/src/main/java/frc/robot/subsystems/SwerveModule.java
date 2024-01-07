package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        m_steerPID = new PIDController(0, 0, 0); // TODO: set theses
        
        m_lastAngle = getState().angle;

        configDrive();
    }

    /**
     * 
     * @param state 
     */
    public void setDesiredState(SwerveModuleState state)  {
        setSpeed(state);
        setAngle(state);
    }

    /**
     * 
     * @return
     */
    private SwerveModuleState getState() {
        if (m_curState != null) {
            m_curState = new SwerveModuleState();
        }

        m_curState.angle = new Rotation2d(getAngle());
        m_curState.speedMetersPerSecond = getSpeed();

        return m_curState;
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
    private void setSpeed(SwerveModuleState state) {
        m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVoltage);
    }

    /**
     * 
     * @return 
     */
    private double getAngle() {
        // 360 degrees in a circle divided by 4096 encoder counts/revolution (CANCoder resolution)
        return m_CANCoder.getPosition() * (360 / 4096);
    }

    /**
     * 
     * @param state
     */
    private void setAngle(SwerveModuleState state) {
        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (DrivetrainConstants.maxSpeed * 0.01)) ? m_lastAngle : state.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        m_steerPID.setSetpoint(angle.getDegrees());
        m_lastAngle = angle;
    }

    /**
     * 
     */
    private void configDrive() {
        // TODO conversions
        m_driveMotor.getEncoder().setVelocityConversionFactor(0); // in meters per second

        // PID loop        
        m_drivePID.setP(0d);
        m_drivePID.setI(0d);
        m_drivePID.setIZone(0d);
        m_drivePID.setD(0d);
        m_drivePID.setFF(0d);

        m_driveMotor.burnFlash();
    }
}
