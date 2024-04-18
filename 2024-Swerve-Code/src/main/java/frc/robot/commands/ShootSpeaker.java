package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class ShootSpeaker extends SequentialCommandGroup {

  public ShootSpeaker(Shooter shooter) {
    addCommands(
      Commands.runOnce(() -> shooter.setSpeed(0.60), shooter),
      new WaitCommand(2),
      Commands.runOnce(() -> shooter.setSpeedConveyor(0.4), shooter),
      new WaitCommand(1.5),
      Commands.runOnce(
        () -> {
          shooter.stopMotor();
          shooter.stopMotorConveyor();
        },
        shooter
      ),
      Commands.runOnce(() -> this.cancel())
    );
  }
}
