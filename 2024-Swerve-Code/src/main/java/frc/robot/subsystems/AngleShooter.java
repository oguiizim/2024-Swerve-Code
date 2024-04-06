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
    pidEncoder.setPositionOffset(0.51);
    pidEncoder.setDutyCycleRange(0.35, 0.90);
    interpolating.put(1.19, 0.65);
    interpolating.put(1.39, 0.67);
    interpolating.put(1.59, 0.69);
    interpolating.put(1.79, 0.72);
    interpolating.put(1.99, 0.745);
    interpolating.put(2.19, 0.76);
    interpolating.put(2.39, 0.765);
    interpolating.put(2.59, 0.77);
    interpolating.put(2.79, 0.774);
    interpolating.put(2.99, 0.776);
    interpolating.put(3.19, 0.7788);
    interpolating.put(3.29, 0.78);

    // interpolating.put(3.59, null);
  }

  public double getAngle() {
    double inter;

    double[] get = LimelightHelpers.getTargetPose_RobotSpace("");
    double tz = get[2];

    inter = interpolating.get(tz);

    if (inter > 1) {
      inter = 0.8;
    } else if (inter < 0.1) {
      inter = 0.5;
    }
    return inter;
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
    double p = SmartDashboard.getNumber("P Gain Shooter", PID.kP);
    double i = SmartDashboard.getNumber("I Gain Shooter", PID.kI);
    double d = SmartDashboard.getNumber("D Gain Shooter", PID.kD);

    if (p != PID.kP) {
      anglePidController.setP(p);
      PID.kP = p;
    }
    if (i != PID.kI) {
      anglePidController.setI(i);
      PID.kI = i;
    }
    if (d != PID.kD) {
      anglePidController.setD(d);
      PID.kD = d;
    }

    double[] get = LimelightHelpers.getTargetPose_RobotSpace("");
    double tz = get[2];
    double outPut = anglePidController.calculate(getPosition());

    outPut = MathUtil.clamp(outPut, -0.4, 0.4);

    setSpeed(outPut);

    SmartDashboard.putNumber("Position", getPosition() * 360);
    SmartDashboard.putNumber("Angle For Shoot", getAngle());
    SmartDashboard.putNumber(
        "Setpoint",
        anglePidController.getSetpoint() * 360);
    SmartDashboard.putNumber("tz", tz);
    SmartDashboard.putNumber("Velocity Angle", outPut);
  }
}