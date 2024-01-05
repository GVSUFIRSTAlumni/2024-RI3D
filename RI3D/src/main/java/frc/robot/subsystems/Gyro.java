package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants;

public class Gyro {
    private Pigeon2 m_gyro = new Pigeon2(Constants.pigeonID);

    public Gyro() {
    }

    /**
     * 
     * @return raw gyro angle in degrees
     */
    public double getGyroAngle() {
        return m_gyro.getCompassHeading();
    }

    /**
     * 
     * @return gyro angle in degrees, clamped [0,360)
     */
    public double getGyroAngleClamped() {
        // %360 -> (-360, 360)
        // +360 -> [0,720)
        // %360 -> [0,360)
        return ((m_gyro.getCompassHeading() % 360) + 360) % 360;
    }
}