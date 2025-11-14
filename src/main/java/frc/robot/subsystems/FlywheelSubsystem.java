
package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSubsystem extends SubsystemBase {
 SparkMax motor1 = new SparkMax(10, MotorType.kBrushless);
//  RelativeEncoder encoder = motor1.getEncoder();

  public FlywheelSubsystem() {}

  @Override
  public void periodic() {}

public void setSpeed(double speed){
  motor1.set(speed);
}

public Command runAtSpeedCommand(double speed){
  return runEnd(() -> motor1.set(speed), () -> setSpeed(0));
}  

public Command runBackSpeedCommand(){
  return runAtSpeedCommand(Constants.backSpeed);
}

public Command runHalfSpeedCommand(){
  return runAtSpeedCommand(-Constants.halfSpeed);
}  

public Command runFullSpeedCommand() {
 return runAtSpeedCommand(Constants.maxSpeed);
}

public Command runIdleSpeedCommand() {
  return runAtSpeedCommand(Constants.idleSpeed);
 }
}