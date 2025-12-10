package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ModuleProperties;

public class SwerveModuleSubsystem {
  Rotation2d offset;

  SparkMax driveMotor;
  SparkMax azimuth;
  SparkClosedLoopController driveController;
  SparkClosedLoopController azimuthController;
  AbsoluteEncoder azimuthEncoder;
  RelativeEncoder driveEncoder;

  public SwerveModuleSubsystem(int driveId, int turnId, Rotation2d offset) {
    this.offset = offset;
    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    azimuth = new SparkMax(turnId, MotorType.kBrushless);

    SparkMaxConfig azimuthConfig = new SparkMaxConfig();
    azimuthConfig.closedLoop.pid(1.5, 0, 0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    azimuthConfig.absoluteEncoder
        .positionConversionFactor(ModuleProperties.kTurningEncoderPositionFactor)
        .velocityConversionFactor(ModuleProperties.kTurningEncoderVelocityFactor)
        .inverted(true);
    azimuth.configure(azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.closedLoop.pid(0.001, 0, 0).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    driveConfig.encoder
        .positionConversionFactor(ModuleProperties.kDrivingEncoderPositionFactor)
        .velocityConversionFactor(ModuleProperties.kDrivingEncoderVelocityFactor);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    azimuthController = azimuth.getClosedLoopController();
    driveController = driveMotor.getClosedLoopController();
    azimuthEncoder = azimuth.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();
  }

  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState offsetState = new SwerveModuleState(state.speedMetersPerSecond, state.angle.plus(offset));
    offsetState.optimize(Rotation2d.fromRotations(azimuthEncoder.getPosition()));
    if (offsetState.speedMetersPerSecond != 0) {
      azimuthController.setReference(offsetState.angle.getRotations(), ControlType.kPosition);
    }
    AngularVelocity desiredAngularVelocity = RadiansPerSecond
        .of(offsetState.speedMetersPerSecond / Units.inchesToMeters(2));
    driveController.setReference(desiredAngularVelocity.in(RPM), ControlType.kVelocity);
  }

  public SwerveModulePosition getPosition() {
    double positionMeters = -ModuleProperties.kWheelDiameterMeters / 2
        * Rotations.of(driveEncoder.getPosition()).in(Radians);
        
    return new SwerveModulePosition(positionMeters, Rotation2d.fromRotations(azimuthEncoder.getPosition()).minus(offset));
  }
}