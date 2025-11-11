package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase{
   SparkMax motor2 = new SparkMax(11, MotorType.kBrushless); 
   RelativeEncoder encoder2 = motor2.getEncoder();

   public IndexSubsystem(){


   }
   @Override
   public void periodic(){


   }
   
public void setSpeed1(double speed1){
    motor2.set(speed1);
  }
  
public Command runAtlaunchCommand(double speed1){
  return runEnd(() ->motor2.set(speed1), () -> setSpeed1(0));
}  
public Command launcherCommand1(){
    return runAtlaunchCommand(Constants.maxSpeed);
}
public Command launcherCommand(){
  return runAtlaunchCommand(-Constants.maxSpeed);
}
public Command idleCommand1(){
  return runAtlaunchCommand(Constants.idleSpeed);
}

}