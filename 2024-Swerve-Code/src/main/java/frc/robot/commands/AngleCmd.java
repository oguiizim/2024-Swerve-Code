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
          
          if(control.getRawButton(Controle.kB)){
               angle.setTarget(0.95);
          } else if(control.getRawButton(Controle.kY)){
               angle.setTarget(0.1);
          } else if(control.getRawButton(Controle.kX)){
               angle.setTarget(0.77);
          } else if(control.getPOV()==0){
               // angle.setTarget(angle.angleForShoot());
               if(angle.tag()){
                    angle.setTarget(0.1);
               } else {
                    angle.setTarget(angle.angleForShoot());
               }
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