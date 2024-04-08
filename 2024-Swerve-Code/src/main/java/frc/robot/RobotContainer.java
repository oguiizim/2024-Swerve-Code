// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controle;
import frc.robot.commands.AngleCmd;
import frc.robot.commands.Gyro;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSource;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.Teleop;
import frc.robot.commands.Auto.MidAuto.mShoot1;
import frc.robot.commands.Auto.MidAuto.mShoot2;
import frc.robot.commands.Auto.MidAuto.mShoot3;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

public class RobotContainer {

  static final SwerveSubsystem swerve = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"));
  public static final Intake subIntake = new Intake();
  public static final AngleShooter subAngle = new AngleShooter();
  public static final Shooter subShooter = new Shooter();

  public static final XboxController driverControl = new XboxController(
      Controle.xboxControle);
  public static final Joystick operatorControl = new Joystick(
      Controle.controle2);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    swerve.setDefaultCommand(
        new Teleop(
            swerve,
            () -> -MathUtil.applyDeadband(driverControl.getLeftY(), Controle.DEADBAND),
            () -> -MathUtil.applyDeadband(driverControl.getLeftX(), Controle.DEADBAND),
            () -> -MathUtil.applyDeadband(driverControl.getRightX(), Controle.DEADBAND),
            driverControl));

    NamedCommands.registerCommand("mShoot1", new mShoot1(subShooter, subAngle));

    NamedCommands.registerCommand("mShoot2", new mShoot2(subShooter));

    NamedCommands.registerCommand("mShoot3", new mShoot3(subShooter));

    NamedCommands.registerCommand("collectConveyor",
        Commands.run(() -> subShooter.collectWithSensor(0.25), subShooter));

    NamedCommands.registerCommand("collect", Commands.runOnce(() -> {
      subAngle.setTarget(0.5);
      subIntake.collectAuto();
    }, subAngle, subIntake));

    NamedCommands.registerCommand("stopIntake", Commands.runOnce(() -> {
      subIntake.stop();
      subShooter.stopMotorConveyor();
    }, subIntake, subShooter));

    NamedCommands.registerCommand("putAngle", Commands.runOnce(
        () -> subAngle.setTarget(subAngle.getAngle()),
        subAngle));

    NamedCommands.registerCommand("angle1", Commands.runOnce(() -> subAngle.setTarget(0.72), subAngle));

    NamedCommands.registerCommand("angle2", Commands.runOnce(() -> subAngle.setTarget(0.76), subAngle));

    NamedCommands.registerCommand("angle3", Commands.runOnce(() -> subAngle.setTarget(0.77), subAngle));

    NamedCommands.registerCommand("angle4", Commands.runOnce(() -> subAngle.setTarget(0.65), subAngle));

    NamedCommands.registerCommand("gyro0", new Gyro(swerve, 0));

    autoChooser = AutoBuilder.buildAutoChooser();

    subShooter.setDefaultCommand(new ShooterCmd(subShooter, operatorControl));
    subAngle.setDefaultCommand(new AngleCmd(subAngle, operatorControl));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverControl, Button.kA.value)
        .onTrue(new InstantCommand(swerve::zeroGyro));

    new Trigger(this::getRight).onTrue(new ShootSpeaker(subShooter));
    new Trigger(this::getLeft).onTrue(new ShootAmp(subShooter));

    new JoystickButton(operatorControl, XboxController.Button.kA.value)
        .whileTrue(
            Commands.startEnd(
                () -> subIntake.collect(),
                () -> subIntake.stop(),
                subIntake));

    new JoystickButton(operatorControl, XboxController.Button.kStart.value)
        .whileTrue(
            Commands.startEnd(
                () -> subIntake.invert(),
                () -> subIntake.stop(),
                subIntake));

    new JoystickButton(
        operatorControl,
        XboxController.Button.kRightBumper.value)
        .onTrue(new ShootSource(subShooter));
  }

  private boolean getRight() {
    if (operatorControl.getRawAxis(Controle.rightTrigger) != 0) {
      return true;
    }
    return false;
  }

  private boolean getLeft() {
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
