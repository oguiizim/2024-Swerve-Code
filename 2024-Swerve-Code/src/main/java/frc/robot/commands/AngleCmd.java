package frc.robot.commands;

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
    angle.setTarget(0.1);
  }

  @Override
  public void execute() {
    if (control.getRawButton(Controle.kB)) {
      angle.setTarget(0.985);
    } else if (control.getRawButton(Controle.kY)) {
      angle.setTarget(0.1);
    } else if (control.getRawButton(Controle.kX)) {
      angle.setTarget(angle.getAngle());
    } else if (control.getRawButton(Controle.kA)) {
      angle.setTarget(0.50);
    } else if (control.getPOV() == 0) { // Colado no speaker
      angle.setTarget(0.65);
    } else if (control.getPOV() == 90) { // Colado na parede
      angle.setTarget(0.806);
    } else if (control.getPOV() == 270) { // Pé do palco traseiro
      angle.setTarget(0.802);
    } else if (control.getPOV() == 180) { // Podium
      angle.setTarget(0.765);
    } else if (control.getRawButton(Controle.kRB)) {
      angle.setTarget(0.735);
    }
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
