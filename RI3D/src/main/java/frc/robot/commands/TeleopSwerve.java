package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve m_swerve;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private DoubleSupplier m_rotationSup;
  private BooleanSupplier m_robotCentricSup;
  private BooleanSupplier m_clutchSupplier;

  private final double TRANSLATION_LIMIT = 4.0;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(TRANSLATION_LIMIT);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(TRANSLATION_LIMIT);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(90);

  public TeleopSwerve(
      Swerve swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier clutchSup) {
    m_swerve = swerve;
    addRequirements(m_swerve);

    m_translationSup = translationSup;
    m_strafeSup = strafeSup;
    m_rotationSup = rotationSup;
    m_robotCentricSup = robotCentricSup;
    m_clutchSupplier = clutchSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(m_rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

    /* Drive */
    m_swerve.drive(
        new Translation2d(translationVal, strafeVal).times(
          (m_clutchSupplier.getAsBoolean() ? Constants.Swerve.throttleRatio : 1) * Constants.Swerve.maxSpeed
        ),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !m_robotCentricSup.getAsBoolean(),
        true);
  }
}
