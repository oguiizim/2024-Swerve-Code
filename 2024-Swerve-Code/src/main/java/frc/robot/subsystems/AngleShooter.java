package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PID;

public class AngleShooter extends SubsystemBase {

     CANSparkMax angle1, angle2;
     DutyCycleEncoder pidEncoder;
     // Encoder piEncoder;
     PIDController anglePidController;
     LimelightHelpers camera = new LimelightHelpers();

     public AngleShooter() {

          anglePidController = new PIDController(PID.kP, PID.kI, PID.kD);
          pidEncoder = new DutyCycleEncoder(0);
          // piEncoder = new Encoder(1, 2, false, EncodingType.k4X);
          angle1 = new CANSparkMax(12, MotorType.kBrushless);
          angle2 = new CANSparkMax(13, MotorType.kBrushless);
          pidEncoder.setPositionOffset(0.51);

          // angle1.setInverted(true);

          // angle2.follow(angle1);
     }

     public double angleShooter() {
          double minValue = 4.1;
          double maxValue = 24.3;
          double value = camera.getTA("");

          double minAngle = 15;
          double maxAngle = 60;
          double proportion = (value - minValue) / (maxValue - minValue);
          double angle = minAngle + (proportion * (maxAngle - minAngle));
          return angle;
     }

     public void reset(){
          pidEncoder.reset();
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

          double outPut = anglePidController.calculate(getPosition());

          outPut = MathUtil.clamp(outPut, -0.5, 0.5);

          // setSpeed(outPut);

          SmartDashboard.putNumber("Position", getPosition());
          SmartDashboard.putNumber("Setpoint", anglePidController.getSetpoint());
          // SmartDashboard.putNumber("Velocity Angle", outPut);
          SmartDashboard.putNumber("Angle", angleShooter());
     }
}
