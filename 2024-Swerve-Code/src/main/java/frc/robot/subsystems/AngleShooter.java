package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PID;

public class AngleShooter extends SubsystemBase {

     CANSparkMax angle1, angle2;
     DutyCycleEncoder pidEncoder;
     PIDController anglePidController;

     public AngleShooter() {

          anglePidController = new PIDController(PID.kP, PID.kI, PID.kD);
          pidEncoder = new DutyCycleEncoder(0);
          angle1 = new CANSparkMax(0, MotorType.kBrushless);
          angle2 = new CANSparkMax(0, MotorType.kBrushless);

          angle2.follow(angle1);
     }

     public void stop() {
          angle1.stopMotor();
     }

     public double getPosition() {
          return pidEncoder.getAbsolutePosition();
     }

     public void setTarget(double setPoint) {
          anglePidController.setSetpoint(setPoint);
     }

     public void setSpeed(double outPut) {
          angle1.set(outPut);
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

          outPut = MathUtil.clamp(outPut, -0.8, 0.8);

          setSpeed(outPut);

          SmartDashboard.putNumber("Abs position", pidEncoder.getAbsolutePosition());
          SmartDashboard.putNumber("Setpoint", anglePidController.getSetpoint());
          SmartDashboard.putNumber("Velocity Angle", outPut);
     }
}
