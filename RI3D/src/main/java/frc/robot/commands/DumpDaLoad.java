// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumppy;

public class DumpDaLoad extends CommandBase {
  private final Dumppy m_dumppy;

  /** Creates a new DumpDaLoad. */
  public DumpDaLoad(Dumppy dumppy) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dumppy = dumppy;
    addRequirements(m_dumppy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dumppy.setGoal(115);
    m_dumppy.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dumppy.setGoal(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
