package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PID;
import frc.robot.LimelightHelpers;

public class AngleShooter extends SubsystemBase {

  CANSparkMax angle1, angle2;
  DutyCycleEncoder pidEncoder;
  PIDController anglePidController;
  LimelightHelpers camera = new LimelightHelpers();
  InterpolatingDoubleTreeMap interpolating = new InterpolatingDoubleTreeMap();

  public AngleShooter() {
    anglePidController = new PIDController(PID.kP, PID.kI, PID.kD);
    pidEncoder = new DutyCycleEncoder(0);
    angle1 = new CANSparkMax(12, MotorType.kBrushless);
    angle2 = new CANSparkMax(13, MotorType.kBrushless);

    angle2.setInverted(false);
    angle1.setInverted(false);
    pidEncoder.setPositionOffset(0.51);
    pidEncoder.setDutyCycleRange(0.05, 0.65);

    interpolating.put(0.98, 0.73);
    interpolating.put(1.18, 0.75);
    interpolating.put(1.38, 0.76);
    interpolating.put(1.58, 0.772);
    interpolating.put(1.78, 0.775);
    interpolating.put(1.98, 0.789);
    interpolating.put(2.18, 0.795);
    interpolating.put(2.38, 0.81);
    interpolating.put(2.58, 0.814);
    interpolating.put(2.78, 0.818);
    // interpolating.put(3.34, 0.772);
    // interpolating.put(3.57, 0.773);
    // interpolating.put(3.81, 0.774);
    // interpolating.put(4.01, 0.775);

  }

  public double getAngle() {
    double inter = 0;

    double[] get = LimelightHelpers.getTargetPose_RobotSpace("");

    if (get.length >= 2) {
      double tz = get[2];

      inter = interpolating.get(tz);

      if (inter > 1) {
        inter = 0.8;
      } else if (inter < 0.1) {
        inter = 0.5;
      }

    }
    return inter;
  }

  private double getTz() {
    double[] get = LimelightHelpers.getTargetPose_RobotSpace("");

    return get.length >= 2 ? get[2] : 0.0;
  }

  public void stop() {
    angle1.stopMotor();
    angle2.stopMotor();
  }

  public double getPosition() {
    return pidEncoder.getAbsolutePosition();
  }

  public void setTarget(double setPoint) {
    anglePidController.setSetpoint(setPoint);
  }

  public void setSpeed(double outPut) {
    angle1.set(outPut);
    angle2.set(outPut);
  }

  @Override
  public void periodic() {
    double outPut = anglePidController.calculate(getPosition());

    outPut = MathUtil.clamp(outPut, -0.4, 0.4);

    setSpeed(outPut);

    SmartDashboard.putNumber("Position", getPosition());
    SmartDashboard.putNumber("Angle For Shoot", getAngle());
    SmartDashboard.putNumber("Setpoint", anglePidController.getSetpoint() * 360);
    SmartDashboard.putNumber("Tz", getTz());
    SmartDashboard.putNumber("Velocity Angle", outPut);
  }
}