package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends SequentialCommandGroup {

  public ShootAmp(Shooter shooter) {
    addCommands(
      Commands.runOnce(() -> shooter.setSpeed(0.18), shooter),
      new WaitCommand(1),
      Commands.runOnce(() -> shooter.setSpeedConveyor(0.3), shooter),
      new WaitCommand(1.5),
      Commands.runOnce(() -> shooter.stopMotor(), shooter),
      Commands.runOnce(() -> shooter.stopMotorConveyor(), shooter),
      Commands.runOnce(() -> this.cancel())
    );
  }
}
