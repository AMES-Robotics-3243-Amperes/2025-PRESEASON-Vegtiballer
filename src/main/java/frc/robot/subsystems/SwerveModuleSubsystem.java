package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModuleSubsystem {
    SparkMax driveMotor;
    SparkMax azimuth;
    AnalogEncoder absulotEncoder;
    Rotation2d ofset;
    PIDController azimuthController;
    SparkClosedLoopController driveController;
    public SwerveModuleSubsystem(int driveId, int turnId,  int encoderId, Rotation2d offset){

    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    azimuth = new SparkMax(turnId, MotorType.kBrushless);
    absulotEncoder = new AnalogEncoder(encoderId);
    this.ofset = offset;
    azimuthController = new PIDController(0.2,0,0);
    azimuthController.enableContinuousInput(-Math.PI, Math.PI);
    driveController = driveMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(0.175, 0, 0);
    driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void setDesiredState(SwerveModuleState state){
    Rotation2d currentRotation = Rotation2d.fromRotations(absulotEncoder.get());
    SwerveModuleState offsetState = new SwerveModuleState();
    offsetState.speedMetersPerSecond = state.speedMetersPerSecond;
    offsetState.angle = state.angle.plus(ofset);
    offsetState.optimize(currentRotation);
    azimuthController.setSetpoint(offsetState.angle.getRadians());
    azimuth.setVoltage(azimuthController.calculate(currentRotation.getRadians()));
    AngularVelocity desiredAngularVelocity = RadiansPerSecond.of(offsetState.speedMetersPerSecond / Units.inchesToMeters(2));
    driveController.setReference(desiredAngularVelocity.in(RPM), ControlType.kVelocity);
    }
}