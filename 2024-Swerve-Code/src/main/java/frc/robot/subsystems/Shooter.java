package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  CANSparkMax shooter1, shooter2, conveyor;

  I2C.Port i2cport = I2C.Port.kOnboard;
  ColorSensorV3 colorSensor = new ColorSensorV3(i2cport);

  public Shooter() {
    shooter1 = new CANSparkMax(9, MotorType.kBrushless);
    shooter2 = new CANSparkMax(11, MotorType.kBrushless);
    conveyor = new CANSparkMax(10, MotorType.kBrushless);

    shooter2.setInverted(true);
    shooter1.setInverted(true);

    conveyor.setInverted(true);
    // shooter2.follow(shooter1);
  }

  public double getProximity() {
    return colorSensor.getProximity();
  }

  public void collectWithSensor(double speed) {
    if (getProximity() > 120) {
      conveyor.stopMotor();
    } else {
      conveyor.set(speed);
    }
  }

  public void setSpeed(double speed) {
    shooter1.set(speed);
    shooter2.set(speed);
  }

  public void stopMotor() {
    shooter1.stopMotor();
    shooter2.stopMotor();
  }

  public void setSpeedConveyor(double speed) {
    conveyor.set(speed);
  }

  public void stopMotorConveyor() {
    conveyor.stopMotor();
  }

  public void stopAll() {
    shooter1.stopMotor();
    shooter2.stopMotor();
    conveyor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Sensor Proximity", colorSensor.getProximity());
  }
}
