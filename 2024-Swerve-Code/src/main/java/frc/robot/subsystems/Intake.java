package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
     
     CANSparkMax frontIntake, backIntake;
     public Intake(){
          frontIntake = new CANSparkMax(0, MotorType.kBrushless);
          backIntake = new CANSparkMax(0, MotorType.kBrushless);

          backIntake.setInverted(true);
     }

     public void collect(){
          frontIntake.set(0.8);
          backIntake.set(0.7);
     }

     public void invert(){
          frontIntake.set(-0.8);
          backIntake.set(-0.7);
     }

     public void stop(){
          frontIntake.stopMotor();
          backIntake.stopMotor();
     }

     @Override
     public void periodic(){
          
     }
}