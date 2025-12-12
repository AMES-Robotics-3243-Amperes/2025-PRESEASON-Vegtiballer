package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WoodSubsystem extends SubsystemBase {
  SparkMax motor3 = new SparkMax(12, MotorType.kBrushless);
  RelativeEncoder encoder2 = motor3.getEncoder();

  public WoodSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    motor3.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    return costomWoodCommand(0.35);
  }

  public Command woodLauncherCommand() {
    return costomWoodCommand(-Constants.slowSpeed*.5);
  }
}
