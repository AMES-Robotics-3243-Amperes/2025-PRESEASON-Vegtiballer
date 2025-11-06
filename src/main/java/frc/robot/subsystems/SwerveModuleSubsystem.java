







// SEMI IMPORTANT:
// May not be finished!
// Code was just copied from my example program with Bryce
// - Sam







package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModule {
    TalonFX driveMotor;
    SparkMax azimuth;
    AnalogEncoder absulotEncoder;
    Rotation2d ofset;
    PIDController azimuthPID;

    public SwerveModule(int driveId, int turnId,  int encoderId, Rotation2d offset){
    driveMotor = new TalonFX(driveId);
    azimuth = new SparkMax(turnId, MotorType.kBrushless);
    absulotEncoder = new AnalogEncoder(encoderId);
    this.ofset = offset;
    azimuthPID = new PIDController(1, 0, 0);
    azimuthPID.enableContinuousInput(-Math.PI, Math.PI);
    
    }
    public void setDesiredState(SwerveModuleState state){
        SwerveModuleState offSetState = new SwerveModuleState();
        offSetState.speedMetersPerSecond = state.speedMetersPerSecond;
        offSetState.angle = state.angle.plus(ofset);
        azimuthPID.setSetpoint(offSetState.angle.getRadians());
    }
}