package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class SwerveDrivetrain extends SubsystemBase {
  private final double robotwidth = Units.inchesToMeters(21);
  private final double robotlength = Units.inchesToMeters(21);
  private final AHRS imu = new AHRS(NavXComType.kMXP_SPI);

  private final SwerveDrivePoseEstimator estimator;
  private Rotation2d gyroRotationOffset = new Rotation2d();
  private boolean megaTagOne = true;

  private final Field2d field = new Field2d();

  public final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(robotlength / 2, robotwidth / 2),
      new Translation2d(robotlength / 2, -robotwidth / 2),
      new Translation2d(-robotlength / 2, robotwidth / 2),
      new Translation2d(-robotlength / 2, -robotwidth / 2));

  private final SwerveModuleSubsystem t_frontleft = new SwerveModuleSubsystem(1, 2, Rotation2d.fromDegrees(90));
  private final SwerveModuleSubsystem t_frontright = new SwerveModuleSubsystem(3, 4, Rotation2d.fromDegrees(180));
  private final SwerveModuleSubsystem t_backleft = new SwerveModuleSubsystem(5, 6, Rotation2d.fromDegrees(0));
  private final SwerveModuleSubsystem t_backright = new SwerveModuleSubsystem(7, 8, Rotation2d.fromDegrees(270));

  public SwerveDrivetrain() {
    LimelightHelpers.setCameraPose_RobotSpace("limelight-two", Units.inchesToMeters(29 / 2 - 1), 0, 0, 0, 0, 0);
    estimator = new SwerveDrivePoseEstimator(kDriveKinematics,
        Rotation2d.fromDegrees(-imu.getYaw()), modulePositions(), new Pose2d());
  }

  public void useMegatagOne() {
    megaTagOne = true;
  }

  public void useMegatagTwo() {
    megaTagOne = false;
    gyroRotationOffset = estimator.getEstimatedPosition().getRotation().minus(Rotation2d.fromDegrees(-imu.getYaw()));
  }

  private SwerveModulePosition[] modulePositions() {
    SwerveModulePosition[] positions = {
        t_frontleft.getPosition(),
        t_frontright.getPosition(),
        t_backleft.getPosition(),
        t_backright.getPosition()
    };
    return positions;
  }

  public void drive(double X, double Y, double omega, boolean robotRelitive) {
    Translation2d speeds = new Translation2d(X, Y);
    if (robotRelitive == false) {
      speeds = speeds.rotateBy(getPosition().getRotation().times(-1));
    }

    ChassisSpeeds chasisSpeeds = new ChassisSpeeds(speeds.getX(), speeds.getY(), omega);
    SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(chasisSpeeds);
    t_frontleft.setDesiredState(states[0]);
    t_frontright.setDesiredState(states[1]);
    t_backleft.setDesiredState(states[2]);
    t_backright.setDesiredState(states[3]);
  }

  public Pose2d getPosition() {
    return estimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    LimelightHelpers.SetRobotOrientation("limelight-two",
        Rotation2d.fromDegrees(-imu.getYaw()).plus(gyroRotationOffset).getDegrees(), -imu.getRate(), 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate estimate = megaTagOne
        ? LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-two")
        : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-two");

    estimator.update(Rotation2d.fromDegrees(-imu.getYaw()), modulePositions());

    boolean useVision = estimate.tagCount != 0 && imu.getRate() < 270;
    if (megaTagOne && estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
      if (estimate.rawFiducials[0].ambiguity > 0.7) {
        useVision = false;
      }
    }

    if (megaTagOne) {
      estimator.setVisionMeasurementStdDevs(VecBuilder.fill(1.5, 1.5, 1.2));
    } else {
      estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 999999));
    }

    if (useVision) {
      estimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    }

    field.setRobotPose(getPosition());
    SmartDashboard.putData(field);
  }
}