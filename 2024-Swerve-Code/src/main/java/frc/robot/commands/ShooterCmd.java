package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command {

  Shooter shooter;
  Intake intake;
  Joystick control;

  public ShooterCmd(Shooter subsystem, Intake subsystem2, Joystick control) {
    this.control = control;
    shooter = subsystem;
    intake = subsystem2;

    addRequirements(subsystem, subsystem2);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (control.getRawButton(1)) {
      intake.collect();
      if (shooter.getProximity() > 100) {
        shooter.stopMotorConveyor();
      } else {
        shooter.setSpeedConveyor(0.33);
      }
    } else {
      shooter.stopMotorConveyor();
      intake.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    shooter.stopMotorConveyor();
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
