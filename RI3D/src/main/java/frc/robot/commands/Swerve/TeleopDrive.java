// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drive;

public class TeleopDrive extends CommandBase {
  /** Creates a new TeleopDrive. */
  private DoubleSupplier m_y; // Used to store the Translation value.
  private DoubleSupplier m_x; // Used to store the Strafe value.
  private DoubleSupplier m_rot; // Used to store the Rotation value.
  private Drive m_drive;

  public TeleopDrive(DoubleSupplier y, DoubleSupplier x, DoubleSupplier rot, Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_y = y;
    m_x = x;
    m_rot = rot;
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var yVal = MathUtil.applyDeadband(m_y.getAsDouble(), Constants.kDeadzone);
    var xVal = MathUtil.applyDeadband(m_x.getAsDouble(), Constants.kDeadzone);
    var rotVal = MathUtil.applyDeadband(m_rot.getAsDouble(), Constants.kDeadzone);

    m_drive.swerve(new Translation2d(xVal, yVal).times(SwerveConstants.maxSpeed), rotVal * SwerveConstants.maxTurningSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
