// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Controle;
import frc.robot.Constants.Dimensoes;
import frc.robot.Constants.Tracao;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * Classe que calcula a partir da entrada do gamepad a saída do swerve
 */
public class Teleop extends Command {

  // Variáveis que guardam nossas funções do gamepad
  DoubleSupplier y;
  DoubleSupplier x;
  DoubleSupplier turn;

  // Objetos necessárias para acessar funções e variáveis
  SwerveSubsystem swerve;
  SwerveController controller;

  // Variáveis que guardam a translação e velocidade angular do swerve
  Translation2d translation;
  double angle;
  double omega;

  XboxController controle1;

  PIDController pid;

  boolean button, button2;

  public Teleop(
      SwerveSubsystem swerve,
      DoubleSupplier y,
      DoubleSupplier x,
      DoubleSupplier turn,
      XboxController controle1) {
    // Aqui atribuimos as funções e subsistema
    this.y = y;
    this.x = x;
    this.turn = turn;
    this.swerve = swerve;
    this.controle1 = controle1;

    pid = new PIDController(10, 0, 0);

    pid.setTolerance(7);

    controller = swerve.getSwerveController(); // Obtemos o controlador do swerve
    // Adiciona a tração como requerimento
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Abaixo calculamos os valores de saída a partir dos nossos inputs
  @Override
  public void execute() {
    double xVelocity = y.getAsDouble() * Tracao.multiplicadorTranslacionalY;
    double yVelocity = x.getAsDouble() * Tracao.multiplicadorTranslacionalX;
    double angVelocity = turn.getAsDouble() * Tracao.multiplicadorRotacional;

    translation = new Translation2d(
        xVelocity * Tracao.MAX_SPEED,
        yVelocity * Tracao.MAX_SPEED);

    omega = controller.config.maxAngularVelocity * angVelocity;

    if (controle1.getRawAxis(Controle.rightTrigger) != 0) {
      translation = new Translation2d(xVelocity * 0.45, yVelocity * 0.45);
    } else if (controle1.getRawAxis(Controle.leftTrigger) != 0) {
      translation = new Translation2d(xVelocity * 4, yVelocity * 4);
    }

    // Caso essa função seja verdadeira a aceleração do robô será limitada
    if (Tracao.accelCorrection) {
      translation = SwerveMath.limitVelocity(
          translation,
          swerve.getFieldVelocity(),
          swerve.getPose(),
          Dimensoes.LOOP_TIME,
          Dimensoes.ROBOT_MASS,
          List.of(Dimensoes.CHASSIS),
          swerve.getSwerveDriveConfiguration());
    }

    // if ((controle1.getXButtonPressed() && !button) || button2) {
    // button2 = true;
    // button = true;
    // } else if (controle1.getXButtonPressed() && button) {
    // button = false;
    // button2 = false;
    // }

    // if (controle1.getXButton() && !button) {
    //   double saida = pid.calculate(Math.toRadians(LimelightHelpers.getTX("")), Math.toRadians(0));
    //   omega = saida;
    //   swerve.drive(new Translation2d(0, 0), omega, Tracao.fieldRelative);

    //   if (omega >= -0.5 && omega <= 0.5) {
    //     button = true;
    //   }
    // } else {
    //   button = false;
    //   swerve.drive(translation, omega, Tracao.fieldRelative);

    // }

    swerve.drive(translation, omega, Tracao.fieldRelative);

    // Aqui temos nossa função definida dentro da classe de subsistema a qual
    // comandara o swerve
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
