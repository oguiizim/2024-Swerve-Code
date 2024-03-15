// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.commands.Teleop;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  static final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public static final XboxController controleXbox = new XboxController(Controle.xboxControle);
  public static final Joystick operatorControl = new Joystick(Controle.controle2);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    swerve.setDefaultCommand(new Teleop(swerve,
        () -> -MathUtil.applyDeadband(controleXbox.getLeftY(), Controle.DEADBAND),
        () -> -MathUtil.applyDeadband(controleXbox.getLeftX(), Controle.DEADBAND),
        () -> -MathUtil.applyDeadband(controleXbox.getRightX(), Controle.DEADBAND), controleXbox));

    // NamedCommands.registerCommand("shootSpeaker",
    //     new InstantCommand(lSubsystem::shootSpeakerAuto, lSubsystem));

    // NamedCommands.registerCommand("shootCondutor", new InstantCommand(lSubsystem::shooterMidCollectDown, lSubsystem));
    // NamedCommands.registerCommand("stopShooter", new InstantCommand(lSubsystem::stop, lSubsystem));
    // NamedCommands.registerCommand("stopCondutor", new InstantCommand(lSubsystem::stopCondutor, lSubsystem));
    // NamedCommands.registerCommand("collect", new InstantCommand(cSubsystem::collect, cSubsystem));
    // NamedCommands.registerCommand("stopIntake", new InstantCommand(cSubsystem::stop, cSubsystem));

    // setPoint positvo = Giro para esquerda
    // setPoint negativo = Giro para direita
    // NamedCommands.registerCommand("gyro-60", new Gyro(swerve, -50));
    // NamedCommands.registerCommand("gyro60", new Gyro(swerve, 55));
    // NamedCommands.registerCommand("gyro54", new Gyro(swerve, 54));
    // NamedCommands.registerCommand("gyro-54", new Gyro(swerve, -54));
    // NamedCommands.registerCommand("gyro0", new Gyro(swerve, 0.5));
    // NamedCommands.registerCommand("gyro90", new Gyro(swerve, 90));
    // NamedCommands.registerCommand("gyro-90", new Gyro(swerve, -90));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(controleXbox, Button.kA.value).onTrue(new InstantCommand(swerve::zeroGyro));

  }

  private boolean getOperatorRightTrigger() {
    if (operatorControl.getRawAxis(Controle.rightTrigger) != 0) {
      return true;
    }
    return false;
  }

  private boolean getOperatorLeftTrigger() {
    if (operatorControl.getRawAxis(Controle.leftTrigger) != 0) {
      return true;
    }
    return false;
  }

  public Command getAutonomousCommand() {
    swerve.zeroGyro();
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
