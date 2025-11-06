package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModule {
    TalonFX driveMotor;
    SparkMax azimuth;
    AnalogEncoder absulotEncoder;
    Rotation2d ofset;
    PIDController azimuthController;
    public SwerveModule(int driveId, int turnId,  int encoderId, Rotation2d offset){
    driveMotor = new TalonFX(driveId);
    azimuth = new SparkMax(turnId, MotorType.kBrushless);
    absulotEncoder = new AnalogEncoder(encoderId);
    this.ofset = offset;
    azimuthController = new PIDController(1,0,0);
    azimuthController.enableContinuousInput(-Math.PI, Math.PI);
    
    }
    public void setDesiredState(SwerveModuleState state){
    Rotation2d currentRotation = Rotation2d.fromRotations(absulotEncoder.get());
    SwerveModuleState offsetState = new SwerveModuleState();
    offsetState.speedMetersPerSecond = state.speedMetersPerSecond;
    offsetState.angle = state.angle.plus(ofset);
    offsetState.optimize(currentRotation);
    azimuthController.setSetpoint(offsetState.angle.getRadians());
    azimuth.setVoltage(azimuthController.calculate(currentRotation.getRadians()));
    AngularVelocity desiredAngularVelocity = RadiansPerSecond.of(offsetState.speedMetersPerSecond/Units.inchesToMeters(2));
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(desiredAngularVelocity);
    driveMotor.setControl(request.withSlot(0));
    }
}