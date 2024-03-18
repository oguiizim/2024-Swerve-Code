package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooter;

public class AngleCmd extends Command {

     Joystick control;
     AngleShooter angle;

     public AngleCmd(AngleShooter subsystem, Joystick control) {
          angle = subsystem;
          this.control = control;

          addRequirements(subsystem);
     }

     @Override
     public void initialize() {
          // angle.setTarget(0);
     }

     @Override
     public void execute() {

          if (control.getRawButton(1)) {
               angle.setTarget(0.1);
          } else if (control.getRawButton(2)) {
               angle.setTarget(0.40);
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
