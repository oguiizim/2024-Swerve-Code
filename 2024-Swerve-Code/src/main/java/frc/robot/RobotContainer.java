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
import frc.robot.Constants.Controle;
import frc.robot.commands.AngleCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.Teleop;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

public class RobotContainer {

  static final SwerveSubsystem swerve = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );
  public static final Intake subIntake = new Intake();
  public static final AngleShooter subAngle = new AngleShooter();
  public static final Shooter subShooter = new Shooter();

  public static final XboxController driverControl = new XboxController(
    Controle.xboxControle
  );
  public static final Joystick operatorControl = new Joystick(
    Controle.controle2
  );

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    swerve.setDefaultCommand(
      new Teleop(
        swerve,
        () ->
          -MathUtil.applyDeadband(driverControl.getLeftY(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(driverControl.getLeftX(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(driverControl.getRightX(), Controle.DEADBAND),
        driverControl
      )
    );

    autoChooser = AutoBuilder.buildAutoChooser();

    subShooter.setDefaultCommand(
      new ShooterCmd(subShooter, subIntake, operatorControl)
    );
    subIntake.setDefaultCommand(
      new ShooterCmd(subShooter, subIntake, operatorControl)
    );
    subAngle.setDefaultCommand(new AngleCmd(subAngle, operatorControl));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverControl, Button.kA.value)
      .onTrue(new InstantCommand(swerve::zeroGyro));
    // new JoystickButton(operatorControl, XboxController.Button.kA.value)
    //   .whileTrue(
    //     Commands.runEnd(
    //       () -> {
    //         subAngle.setTarget(0.50);
    //         subIntake.collect();
    //         subShooter.setSpeedConveyor(0.35);
    //         get();
    //       },
    //       () -> {
    //         subIntake.stop();
    //         subShooter.stopMotorConveyor();
    //       },
    //       subIntake,
    //       subShooter,
    //       subAngle
    //     )
    //   );

    // new JoystickButton(operatorControl, XboxController.Button.kStart.value)
    //   .whileTrue(
    //     Commands.runEnd(
    //       () -> {
    //         subIntake.invert();
    //       },
    //       () -> {
    //         subIntake.stop();
    //       },
    //       subIntake
    //     )
    //   );
  }

  private void get() {
    if (subShooter.getProximity() > 100) {
      subShooter.stopMotorConveyor();
    }
  }

  public Command getAutonomousCommand() {
    swerve.zeroGyro();
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
