package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

     CANSparkMax shooter1, shooter2, conveyor;

     I2C.Port i2cport = I2C.Port.kOnboard;
     ColorSensorV3 colorSensor = new ColorSensorV3(i2cport);

     public Shooter() {

          shooter1 = new CANSparkMax(0, MotorType.kBrushless);
          shooter2 = new CANSparkMax(0, MotorType.kBrushless);
          conveyor = new CANSparkMax(0, MotorType.kBrushless);

          conveyor.setInverted(true);
          shooter2.follow(shooter1);
     }

     public boolean getProximity() {
          if (colorSensor.getProximity() > 110) {
               return true;
          }
          return false;
     }

     public void setSpeed(double speed) {
          shooter1.set(speed);
     }

     public void stopMotor() {
          shooter1.stopMotor();
     }

     public void setSpeedConveyor(double speed) {
          conveyor.set(speed);
     }

     public void stopMotorConveyor() {
          conveyor.stopMotor();
     }

     @Override
     public void periodic() {
          SmartDashboard.putNumber("Shooter Velocity", shooter1.getEncoder().getVelocity());
          SmartDashboard.putNumber("Conveyor Velocity", conveyor.getEncoder().getVelocity());
          SmartDashboard.putNumber("Sensor Proximity", colorSensor.getProximity());
     }
}