package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {
  SparkMax motor2 = new SparkMax(11, MotorType.kBrushless);
  RelativeEncoder encoder2 = motor2.getEncoder();

  public IndexSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    motor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {

  }

  public void setSpeed(double speed1) {
    motor2.set(speed1);
  }

  public Command runAtSpeed(double speed) {
    return runEnd(() -> motor2.set(speed), () -> setSpeed(0));
  }

  public Command runFullSpeedCommand() {
    return runAtSpeed(Constants.maxSpeed);
  }

  public Command runHalfSpeedCommand() {
    return runAtSpeed(Constants.slowSpeed);
  }

  public Command runBackSpeedCommand() {
    return runAtSpeed(-Constants.slowSpeed);
  }

  public Command idleCommand() {
    return runAtSpeed(Constants.idleSpeed);
  }

}