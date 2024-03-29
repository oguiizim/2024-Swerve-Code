package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Controle;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command {

  Shooter shooter;
  Joystick control = new Joystick(1);

  public ShooterCmd(Shooter subsystem, Joystick control) {
    this.control = control;
    shooter = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (control.getRawButton(1)) {
         shooter.setSpeedConveyor(0.23);
      if (shooter.getProximity() > 100) {
        shooter.stopMotorConveyor();
      } 
    } else {
      shooter.stopMotorConveyor();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    shooter.stopMotorConveyor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}