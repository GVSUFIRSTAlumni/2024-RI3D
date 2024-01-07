package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule {
    private final CANSparkMax m_driveMotor, m_steerMotor;

    private final RelativeEncoder m_driveEncoder, m_steerEncoder;

    private final SparkPIDController m_drivePID;
    private final PIDController m_steerPID;
    private final SimpleMotorFeedforward m_driveFF;

    private final WPI_CANCoder m_CANCoder;

    private final double m_encoderOffset;

    private SwerveModuleState m_curState;

    private final int m_swerveID;

    /**
     * 
     * @param driveID       CAN ID of the drive motor
     * @param steerID       CAN ID of the steer motor
     * @param encoderID     CAN ID of the CANCoder
     * @param encoderOffset starting encoder position
     */
    public SwerveModule(final int driveID, final int steerID, final int encoderID, final double encoderOffset, int swerveID) {
        m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);       
        m_CANCoder = new WPI_CANCoder(encoderID);
        m_encoderOffset = encoderOffset;

        m_driveEncoder = m_driveMotor.getEncoder();
        m_steerEncoder = m_steerMotor.getEncoder();
        
        m_driveFF = new SimpleMotorFeedforward(0.667, 2.44, 0.27);

        m_steerEncoder.setPositionConversionFactor((1/12.8) * 2 * Math.PI);
        m_driveEncoder.setVelocityConversionFactor(((Units.inchesToMeters(4) * Math.PI) / 6.75) / 60);

        m_drivePID = m_driveMotor.getPIDController();
        // m_drivePID = new PIDController(0.1, 0, 0);
        m_steerPID = new PIDController(2, 0, 0); // TODO: set theses
        m_steerPID.enableContinuousInput(-Math.PI, Math.PI);

        m_swerveID = swerveID;

        resetEncoders();

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
        return m_driveEncoder.getVelocity();
    }

    /**
     * 
     * @param state
     */
    public void setSpeed(SwerveModuleState state) {
        m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0, m_driveFF.calculate(state.speedMetersPerSecond));
        // m_drivePID.setSetpoint(state.speedMetersPerSecond);
    }

    /**
     * 
     * @return 
     */
    public double getAngle() {
        // 360 degrees in a circle divided by 4096 encoder counts/revolution (CANCoder resolution)
        // return (m_CANCoder.getAbsolutePosition() * 360 / 4096) - m_encoderOffset.getDegrees();
        return m_steerEncoder.getPosition();
    }

    /**
     * 
     * @param state
     */
    private void setAngle(SwerveModuleState state) {
        //Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (DrivetrainConstants.maxSpeed * 0.01)) ? m_lastAngle : state.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = state.angle;
        m_steerPID.setSetpoint(angle.getRadians());
    }

    private double getAbsEncoderPos() {
        return Units.degreesToRadians(m_CANCoder.getAbsolutePosition() - m_encoderOffset);
    }

    private void resetEncoders() {
        m_driveEncoder.setPosition(0);
        m_steerEncoder.setPosition(getAbsEncoderPos());
    }

    private double getDistance() {
        return m_driveMotor.getEncoder().getPosition();
    }

    /**
     * 
     */
    private void configDrive() {
        m_driveMotor.restoreFactoryDefaults();

        m_driveMotor.getEncoder().setVelocityConversionFactor((((Units.inchesToMeters(4) * Math.PI) / 6.75) / 60)); // in meters per second
        m_driveMotor.getEncoder().setPositionConversionFactor((((Units.inchesToMeters(4) * Math.PI) / 6.75)));

        m_drivePID.setP(DrivetrainConstants.DriveParams.kP);
        m_drivePID.setI(DrivetrainConstants.DriveParams.kI);
        m_drivePID.setD(DrivetrainConstants.DriveParams.kD);
        // m_drivePID.setFF(DrivetrainConstants.DriveParams.kFF);

        m_driveMotor.setIdleMode(DrivetrainConstants.DriveParams.kIdleMode);

        m_driveMotor.burnFlash();
    }

    public void updateSteer() {
        // double driveV = m_drivePID.calculate(getSpeed()) + m_driveFF.calculate(getSpeed());
        m_steerMotor.setVoltage(m_steerPID.calculate(getAngle()));
        // m_driveMotor.setVoltage(driveV);
        if(m_swerveID == 0) {
            // System.out.println("Current setpoint: " + m_drivePID.getSetpoint());
            // System.out.println("Current speed :" + getSpeed());
            // System.out.println(driveV);
        }
    }
}
