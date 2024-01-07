package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class Gyro {
    private Pigeon2 m_gyro = new Pigeon2(Constants.pigeonID);

    public Gyro() {
    }

    /**
     * 
     * @return raw gyro angle in degrees
     */
    public Rotation2d getGyroAngle() {
        return new Rotation2d(m_gyro.getAbsoluteCompassHeading());
    }

    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * 
     * @return gyro angle in degrees, clamped [0,360)
     */
    public double getGyroAngleClamped() {
        // %360 -> (-360, 360)
        // +360 -> [0,720)
        // %360 -> [0,360)
        // return ((m_gyro.getCompassHeading() % 360) + 360) % 360;
        return m_gyro.getAbsoluteCompassHeading();
    }
}