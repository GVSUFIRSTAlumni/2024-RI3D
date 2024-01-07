package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Wheel extends SubsystemBase {
    private final CANSparkMax m_Drive, m_Steer;
    private final SparkPIDController m_DriveController, m_SteerController;
    private final RelativeEncoder m_DriveEncoder, m_SteerEncoder;
    private final WPI_CANCoder m_CANCoder;

    public enum WheelPos {
        FRONTLEFT, FRONTRIGHT, BACKLEFT, BACKRIGHT
    };

    private final WheelPos m_WheelPos;

    public Wheel(WheelPos wheelPos) {
        m_WheelPos = wheelPos;
        switch (m_WheelPos) {
            case FRONTLEFT:
                m_Drive = new CANSparkMax(SwerveConstants.frontLeftDriveID, MotorType.kBrushless);
                m_Steer = new CANSparkMax(SwerveConstants.frontLeftSteerID, MotorType.kBrushless);
                m_CANCoder = new WPI_CANCoder(SwerveConstants.frontLeftCANCoderID);
                break;
            case FRONTRIGHT:
                m_Drive = new CANSparkMax(SwerveConstants.frontRightDriveID, MotorType.kBrushless);
                m_Steer = new CANSparkMax(SwerveConstants.frontRightSteerID, MotorType.kBrushless);
                m_CANCoder = new WPI_CANCoder(SwerveConstants.frontRightCANCoderID);
                break;
            case BACKLEFT:
                m_Drive = new CANSparkMax(SwerveConstants.backLeftDriveID, MotorType.kBrushless);
                m_Steer = new CANSparkMax(SwerveConstants.backLeftSteerID, MotorType.kBrushless);
                m_CANCoder = new WPI_CANCoder(SwerveConstants.backLeftCANCoderID);
                break;
            case BACKRIGHT:
                m_Drive = new CANSparkMax(SwerveConstants.backRightDriveID, MotorType.kBrushless);
                m_Steer = new CANSparkMax(SwerveConstants.backRightSteerID, MotorType.kBrushless);
                m_CANCoder = new WPI_CANCoder(SwerveConstants.backRightCANCoderID);
                break;
            default:
                // if we get here I will cry
                m_Drive = null;
                m_Steer = null;
                m_CANCoder = null;
                System.out.println("ILLEGAL SPOT REACHED - STOP!");
                System.exit(1);
        }

        m_DriveController = m_Drive.getPIDController();
        m_SteerController = m_Steer.getPIDController();

        m_DriveEncoder = m_Drive.getEncoder();
        m_SteerEncoder = m_Drive.getEncoder();

        stop();

        m_Drive.restoreFactoryDefaults();
        m_DriveController.setP(SwerveConstants.DriveParams.kP);
        m_DriveController.setI(SwerveConstants.DriveParams.kI);
        m_DriveController.setD(SwerveConstants.DriveParams.kD);
        m_DriveController.setFF(SwerveConstants.DriveParams.kFF);
        m_DriveEncoder.setVelocityConversionFactor(SwerveConstants.DriveParams.kVelocityCF);
        m_Drive.burnFlash();

        m_Steer.restoreFactoryDefaults();
        m_SteerController.setP(SwerveConstants.SteerParams.kP);
        m_SteerController.setI(SwerveConstants.SteerParams.kI);
        m_SteerController.setD(SwerveConstants.SteerParams.kD);
        m_SteerController.setFF(SwerveConstants.SteerParams.kFF);
        m_SteerEncoder.setPositionConversionFactor(SwerveConstants.SteerParams.kPositionCF);
        m_Steer.burnFlash();
    }

    public void setDesiredState(SwerveModuleState state)  {
        setSpeed(state);
        setAngle(state);
    }

    private void setSpeed(SwerveModuleState state) {
        m_DriveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }

    private void setAngle(SwerveModuleState state) {
        m_SteerController.setReference(state.angle.getDegrees(), ControlType.kPosition);
    }

    private void stop() {
        m_Drive.stopMotor();
        m_Steer.stopMotor();
    }

    public WheelPos getPos() {
        return m_WheelPos;
    }
}
