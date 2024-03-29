// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class X_Lock extends CommandBase {
  private final Swerve m_swerve;
  SwerveModuleState[] states = new SwerveModuleState[4];

  
  /** Creates a new X_Lock. */
  public X_Lock(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SwerveModuleState fourtyFive = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
    // SwerveModuleState oneThirtyFive = new SwerveModuleState(0.0, Rotation2d.fromDegrees(135));

    // states[0] = fourtyFive;
    // states[1] = oneThirtyFive;
    // states[2] = fourtyFive;
    // states[3] = oneThirtyFive;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.lock();
    // m_swerve.setModuleStates(states);
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
