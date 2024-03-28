package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
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
          
          if(control.getRawButton(2)){
               angle.setTarget(0.45);
          } else if(control.getRawButton(3)){
               angle.setTarget(0.1);
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
