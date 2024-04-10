package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Controle;
import frc.robot.Constants.PID;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroLimelight extends Command {

  PIDController anglePIDController;

  SwerveSubsystem swerve;

  double setPoint;

  XboxController control = new XboxController(0);

  public GyroLimelight(SwerveSubsystem subsystem, double setPoint) {
    swerve = subsystem;
    this.setPoint = setPoint;
    anglePIDController = new PIDController(
        10,
        0,
        0);
    anglePIDController.enableContinuousInput(-54, 54);

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // NetworkTableInstance.getDefault().getTable("").getEntry("id").getDouble(<default
    // value>);
    double id = NetworkTableInstance.getDefault().getTable("").getEntry("tid").getDouble(0);
    // if (id == 4) {

    // }

    double outPut = anglePIDController.calculate(
        Math.toRadians(LimelightHelpers.getTX("")),
        Math.toRadians(setPoint));

    swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, outPut));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {

    double tx = LimelightHelpers.getTX("");
    if (tx > -1 && tx < 1) {
      return true;
    } else if (control.getRawButton(Controle.kY)) {
      return true;
    }

    return false;

  }
}
