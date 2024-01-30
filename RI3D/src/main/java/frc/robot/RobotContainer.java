// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.auto.exampleAuto;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final XboxController driver = new XboxController(0);

  /* Subsystems */
  private final Swerve m_swerve = new Swerve();
  private final Dumppy m_dumppy = new Dumppy();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setFPS(60);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_swerve.setDefaultCommand(
      new TeleopSwerve(
        m_swerve,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX(),
        () -> driver.getLeftBumper(),
        () -> !driver.getRightBumper()
      )
    );

    /* Driver Buttons */
    new JoystickButton(driver, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
    new JoystickButton(driver, XboxController.Button.kB.value).onTrue(new CarryDaLoad(m_dumppy));
    new JoystickButton(driver, XboxController.Button.kX.value).whileTrue(new X_Lock(m_swerve));
    new JoystickButton(driver, XboxController.Button.kBack.value).onTrue(new InstantCommand(() -> m_swerve.resetSteeringEncoders()));

    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new DumpDaLoad(m_dumppy));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(m_swerve);
  }
}
