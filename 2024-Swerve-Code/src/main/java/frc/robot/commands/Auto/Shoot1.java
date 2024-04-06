package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Shooter;

public class Shoot1 extends SequentialCommandGroup {

     public Shoot1(Shooter shooter, AngleShooter angle) {
          addCommands(
                    Commands.runOnce(() -> angle.setTarget(.82), angle),
                    Commands.runOnce(() -> shooter.setSpeed(0.65), shooter),
                    new WaitCommand(2.5),
                    Commands.runOnce(() -> shooter.setSpeedConveyor(1), shooter),
                    new WaitCommand(2),
                    Commands.runOnce(() -> shooter.stopAll(), shooter),
                    Commands.runOnce(() -> this.cancel()));
     }
}
