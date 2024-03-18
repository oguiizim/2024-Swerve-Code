package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command {

     Shooter shooter;
     Joystick control;

     public ShooterCmd(Shooter subsystem, Joystick control) {
          this.control = control;
          shooter = subsystem;

          addRequirements(subsystem);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {

          if (control.getRawButton(1)) {
               shooter.setSpeedConveyor(0.3);
               if (shooter.getProximity()) {
                    shooter.stopMotorConveyor();
               }
          } else if (control.getRawAxis(3) != 0) {
               shooter.setSpeed(0.8);
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
