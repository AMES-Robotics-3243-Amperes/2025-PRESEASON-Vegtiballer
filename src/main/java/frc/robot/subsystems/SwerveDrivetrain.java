package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {
    private final double robotwidth = Units.inchesToMeters(21);
    private final double robotlength = Units.inchesToMeters(21);
    private final AHRS imu = new AHRS(NavXComType.kMXP_SPI);

    
    public final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(robotlength /2, robotwidth /2),
        new Translation2d(robotlength /2, -robotwidth /2),
        new Translation2d(-robotlength /2, robotwidth /2),
        new Translation2d(-robotlength /2, -robotwidth /2)
    );



    private final SwerveModuleSubsystem t_frontleft = new SwerveModuleSubsystem(1, 2 ,Rotation2d.fromDegrees(90));
    private final SwerveModuleSubsystem t_frontright = new SwerveModuleSubsystem(3, 4, Rotation2d.fromDegrees(180));
    private final SwerveModuleSubsystem t_backleft = new SwerveModuleSubsystem(5, 6, Rotation2d.fromDegrees(0));
    private final SwerveModuleSubsystem t_backright = new SwerveModuleSubsystem(7, 8, Rotation2d.fromDegrees(270));
    
public void drive(double X,double Y, double omega, boolean robotRelitive){
    Translation2d speeds = new Translation2d(X,Y);
    if (robotRelitive == false ){
        speeds = speeds.rotateBy(Rotation2d.fromDegrees(imu.getYaw()));
    }

    ChassisSpeeds chasisSpeeds = new ChassisSpeeds(speeds.getX(), speeds.getY(),omega);
SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(chasisSpeeds);
t_frontleft.setDesiredState(states[0]);
t_frontright.setDesiredState(states[1]);
t_backleft.setDesiredState(states[2]);
t_backright.setDesiredState(states[3]);
}
}