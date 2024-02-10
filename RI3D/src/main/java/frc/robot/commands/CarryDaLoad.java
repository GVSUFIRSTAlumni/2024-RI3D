// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Dumppy;

public class CarryDaLoad extends InstantCommand {
  private final Dumppy m_dumppy;

  public CarryDaLoad(Dumppy dumppy) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dumppy = dumppy;
    addRequirements(m_dumppy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dumppy.setGoal(55);
    m_dumppy.enable();
  }
}
