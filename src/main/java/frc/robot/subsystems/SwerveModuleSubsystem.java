package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
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
    SparkClosedLoopController driveController;
    SparkClosedLoopController azimuthController;
    AbsoluteEncoder azimuthEncoder;
    public SwerveModuleSubsystem(int driveId, int turnId){
    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    azimuth = new SparkMax(turnId, MotorType.kBrushless);
 
    azimuthController = azimuth.getClosedLoopController();
    driveController = driveMotor.getClosedLoopController();
    azimuthEncoder = azimuth.getAbsoluteEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(0.175, 0, 0);
    driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void setDesiredState(SwerveModuleState state){
    SwerveModuleState offsetState = state;
    offsetState.optimize(Rotation2d.fromRadians(azimuthEncoder.getPosition()));
    azimuthController.setReference(offsetState.angle.getRadians(),ControlType.kPosition);
    AngularVelocity desiredAngularVelocity = RadiansPerSecond.of(offsetState.speedMetersPerSecond / Units.inchesToMeters(2));
    driveController.setReference(desiredAngularVelocity.in(RPM), ControlType.kVelocity);
    }
}