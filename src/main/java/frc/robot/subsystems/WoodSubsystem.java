package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WoodSubsystem extends SubsystemBase {
  SparkMax motor3 = new SparkMax(12, MotorType.kBrushless);
  RelativeEncoder encoder2 = motor3.getEncoder();

  public WoodSubsystem() {
  }

  @Override
  public void periodic() {
  }

  public void setSpeed2(double speed2) {
    motor3.set(speed2);
  }

  public Command costomWoodCommand(double speed2) {
    return runEnd(() -> motor3.set(speed2), () -> setSpeed2(0));
  }

  public Command WoodCommand1() {
    return costomWoodCommand(Constants.maxSpeed);
  }

  public Command woodLauncherCommand() {
    return costomWoodCommand(-Constants.maxSpeed);
  }
}
