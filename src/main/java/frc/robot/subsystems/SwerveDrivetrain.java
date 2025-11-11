package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {
    private final double robotwidth = Units.inchesToMeters(32);
    private final double robotlength = Units.inchesToMeters(32);
    
    public final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(robotlength /2, robotwidth /2),
        new Translation2d(robotlength /2, -robotwidth /2),
        new Translation2d(-robotlength /2, robotwidth /2),
        new Translation2d(-robotlength /2, -robotlength /2)
    );



    private final SwerveModuleSubsystem t_frontleft = new SwerveModuleSubsystem(0, 0, 0, Rotation2d.fromDegrees(0) );
    private final SwerveModuleSubsystem t_frontright = new SwerveModuleSubsystem(1, 1, 1, Rotation2d.fromDegrees(1) );
    private final SwerveModuleSubsystem t_backleft = new SwerveModuleSubsystem(2, 2, 2, Rotation2d.fromDegrees(2) );
    private final SwerveModuleSubsystem t_backright = new SwerveModuleSubsystem(3, 3, 3, Rotation2d.fromDegrees(3) );
public void drive(double X,double Y, double omega){
    ChassisSpeeds speed = new ChassisSpeeds(X,Y,omega);
SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(speed);
t_frontleft.setDesiredState(states[0]);
t_frontright.setDesiredState(states[1]);
t_backleft.setDesiredState(states[2]);
t_backright.setDesiredState(states[3]);
}
}