package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Controle;
import frc.robot.subsystems.AngleShooter;

public class AngleCmd extends Command {

     Joystick control = new Joystick(1);

     // PIDController anglePidController;

     AngleShooter angle;

     public AngleCmd(AngleShooter subsystem, Joystick control) {
          angle = subsystem;
          this.control = control;
          // anglePidController = new PIDController(0.5, 0, 0);

          addRequirements(subsystem);
     }

     @Override
     public void initialize() {
          angle.setTarget(0.56);
          // angle.reset();
     }

     @Override
     public void execute() {
          
          if (control.getRawButton(Controle.kB)) {
               angle.setTarget(50 / 360);
          } else if (control.getPOV() == 180) {
               angle.setSpeed(-0.5);
          } else if (control.getPOV() == 0) {
               angle.setSpeed(0.5);
          } else {
               angle.stop();
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
