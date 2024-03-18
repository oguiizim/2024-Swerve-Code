package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PID;
import frc.robot.subsystems.SwerveSubsystem;

public class Gyro extends Command {

    PIDController anglePIDController;

    SwerveSubsystem swerve;

    double setPoint;

    XboxController control;

    public Gyro(SwerveSubsystem subsystem, double setPoint, XboxController control) {
        swerve = subsystem;
        this.setPoint = setPoint;
        this.control = control;
        anglePIDController = new PIDController(PID.angleAutoPID.p, PID.angleAutoPID.i, PID.angleAutoPID.d);
        anglePIDController.enableContinuousInput(-180, 180);
        // anglePIDController.setTolerance(0.1);

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double outPut = anglePIDController.calculate(swerve.getHeading().getRadians(), Math.toRadians(setPoint));
        swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, outPut));

        if (control.getPOV() == 0) {
            setPoint = 0;
        } else if (control.getPOV() == 270) {
            setPoint = -90;
        } else if (control.getPOV() == 90) {
            setPoint = 90;
        } else if (control.getPOV() == 180) {
            setPoint = 180;
        } else if (control.getXButton()) {
            setPoint = 60;
        } else if (control.getBButton()) {
            setPoint = -60;
        }

    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return anglePIDController.atSetpoint();
    }
}
