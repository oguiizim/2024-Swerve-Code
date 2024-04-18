package frc.robot.commands.AngleCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooter;

public class Collect extends Command {

     AngleShooter angle;

     public Collect(AngleShooter subsystem) {
          angle = subsystem;

          addRequirements(subsystem);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          angle.setTarget(0.53);
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
