package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends SequentialCommandGroup {
     public ShootAmp(Shooter shooter) {
          addCommands(
             Commands.runOnce(() -> shooter.setSpeed(0.2), shooter),  
             new WaitCommand(1),  
             Commands.runOnce(() -> shooter.setSpeedConveyor(0.65), shooter),
             new WaitCommand(1.5),  
             Commands.runOnce(() -> {
               shooter.stopMotor(); 
               shooter.stopMotorConveyor();
          }, shooter),
             Commands.runOnce(() -> this.cancel())
          );     
     }
}