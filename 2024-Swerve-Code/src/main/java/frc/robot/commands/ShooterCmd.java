package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
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
     public void initialize() {
     }

     @Override
     public void execute() {

          if (control.getRawAxis(Controle.rightTrigger) != 0) {
               try {
                    shooter.shootSpeaker(1000, 700);
               } catch (InterruptedException e) {
                    e.printStackTrace();
               }

          } else if (control.getRawAxis(Controle.leftTrigger) != 0) {
               try {
                    shooter.shootAmp();
               } catch (InterruptedException e) {
                    e.printStackTrace();
               }

          } else if (control.getRawButton(Controle.kA)) {
               shooter.setSpeedConveyor(0.5);
               if (shooter.getProximity() > 100) {
                    shooter.stopMotorConveyor();
               }
          } else {
               shooter.stopMotor();
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
