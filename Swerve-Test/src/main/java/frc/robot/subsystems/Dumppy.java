// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DumppyConstants;

public class Dumppy extends ProfiledPIDSubsystem {
  private final CANSparkMax m_dumppy;

  private final RelativeEncoder m_encoder;

  /** Creates a new Dumppy. */
  public Dumppy() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0.3,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(3600 * Constants.DumppyConstants.dumpySpeed,
             3000 * Constants.DumppyConstants.dumpySpeed)));

    m_dumppy = new CANSparkMax(DumppyConstants.kDumppyMotorID, MotorType.kBrushless);

    m_encoder = m_dumppy.getEncoder();
    m_encoder.setPositionConversionFactor((1.0 / 48.0) * 360.0);
    m_encoder.setPosition(0);

    m_controller.setGoal(0);
    this.enable();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_dumppy.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // System.out.println(m_encoder.getPosition());
    super.periodic();
  }
}
