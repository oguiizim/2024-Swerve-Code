package frc.robot.commands;

import javax.sound.sampled.Control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Controle;
import frc.robot.subsystems.AngleShooter;

public class AngleCmd extends Command {

  Joystick control = new Joystick(1);

  AngleShooter angle;

  public AngleCmd(AngleShooter subsystem, Joystick control) {
    angle = subsystem;
    this.control = control;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    angle.setTarget(0.85);
  }

  @Override
  public void execute() {
    if (control.getRawButton(Controle.kA)) {
      angle.setTarget(0.54);
    } else if (control.getRawButton(Controle.kB)) {
      angle.setTarget(0.5);
    } else if (control.getRawButton(Controle.kY)) {
      angle.setTarget(0.85);
}

    // if (control.getRawButton(2)) {
    // angle.setSpeed(0.1);
    // } else if (control.getRawButton(3)) {
    // angle.setSpeed(-0.1);
    // }
  }

  @Override
  public void end(boolean interrupted) {
    angle.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
