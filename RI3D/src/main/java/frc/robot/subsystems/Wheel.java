package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Wheel {
    public enum WheelPos {
        FRONTLEFT, FRONTRIGHT, BACKLEFT, BACKRIGHT
    };

    private final CANSparkMax m_driveMotor, m_steerMotor;
    private final SparkPIDController m_drivePID, m_steerPID;
    private final WheelPos m_pos;

    public Wheel(final WheelPos position, final int driveID, final int steerID) {
        m_pos = position;
        m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);

        m_drivePID = m_driveMotor.getPIDController();
        m_steerPID = m_steerMotor.getPIDController();

        m_driveMotor.stopMotor();
        m_steerMotor.stopMotor();

        // TODO configure motor parameters (current, PID, etc)
        m_drivePID.setP(0d);
        m_drivePID.setI(0d);
        m_drivePID.setD(0d);
        m_drivePID.setFF(0d);
        m_drivePID.setOutputRange(0d, 0d);

        m_steerPID.setP(0d);
        m_steerPID.setI(0d);
        m_steerPID.setD(0d);
        m_steerPID.setFF(0d);
        m_steerPID.setOutputRange(0d, 0d);

        m_driveMotor.burnFlash();
        m_steerMotor.burnFlash();
    }

    public WheelPos getPos() {
        return m_pos;
    }

    public double getDriveRotations() {
        return m_driveMotor.getEncoder().getPosition();
    }

    public double getSteerRotations() {
        return m_steerMotor.getEncoder().getPosition();
    }

    public void setDriveVelocity() {
        
    }
}
