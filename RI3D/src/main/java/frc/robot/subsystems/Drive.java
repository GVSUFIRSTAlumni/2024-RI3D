package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Wheel.WheelPos;

public class Drive extends SubsystemBase {
    private Wheel[] m_Wheels;

    private final Gyro m_Gyro;

    public Drive(Gyro gyro) {
        m_Gyro = gyro;

        m_Wheels = new Wheel[] {
            new Wheel(WheelPos.FRONTLEFT),
            new Wheel(WheelPos.FRONTRIGHT),
            new Wheel(WheelPos.BACKLEFT),
            new Wheel(WheelPos.BACKRIGHT)
        };
    }

    /**
     * 
     * @param translation   linear movement (meters/sec)
     * @param rotation      rotational magnitude (radians/sec)
     * @param fieldOriented if true, swerve with respect to the bot
     */
    public void swerve(Translation2d translation, Double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            new Rotation2d(m_Gyro.getGyroAngleClamped())
        );

        SwerveModuleState[] moduleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);

        for (var curWheel : m_Wheels) {
            switch (curWheel.getPos()) {
                case FRONTLEFT:
                    curWheel.setDesiredState(moduleStates[0]);
                    break;
                case FRONTRIGHT:
                    curWheel.setDesiredState(moduleStates[1]);
                    break;
                case BACKLEFT:
                    curWheel.setDesiredState(moduleStates[2]);
                    break;
                case BACKRIGHT:
                    curWheel.setDesiredState(moduleStates[3]);
                    break;
            }
        }
    }

    @Override
    public void periodic() {}
}
