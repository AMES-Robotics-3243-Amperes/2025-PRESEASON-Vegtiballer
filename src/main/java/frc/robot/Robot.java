package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FlywheelJoysticCommand;
import frc.robot.commands.SwerveDriveTeleopCommand;
import frc.robot.commands.SwerveDriveToPointCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.WoodSubsystem;

public class Robot extends TimedRobot {
  // WoodSubsystem wood = new WoodSubsystem();
  FlywheelSubsystem flywheel = new FlywheelSubsystem();
  IndexSubsystem Index = new IndexSubsystem();
  CommandXboxController controller = new CommandXboxController(0);
  FlywheelJoysticCommand flywheelJoysticCommand = new FlywheelJoysticCommand(controller, flywheel);
  SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  WoodSubsystem wood = new WoodSubsystem();

  public Robot() {
    /*
     * flywheel.setDefaultCommand(flywheelJoysticCommand);
     * Trigger leftx = new Trigger(() -> controller.getLeftX()<0);
     * controller.a().and(leftx).onTrue(
     * flywheel.runFullSpeedCommand()
     * );/*
     */
    // flywheel.setDefaultCommand(flywheel.runIdleSpeedCommand());

    controller.a().whileTrue(
        Index.runHalfSpeedCommand().alongWith(flywheel.runBackSpeedCommand()).withTimeout(Seconds.of(2)));
    // controller.x().whileTrue(Index.runBackSpeedCommand()).withTimeout(Seconds.of(3));
    controller.b().whileTrue(
        new SwerveDriveToPointCommand(drivetrain, new Pose2d(15, 0.7, Rotation2d.fromDegrees(0)))
            .alongWith(
                flywheel.runBackSpeedCommand().withTimeout(Seconds.of(0.5)).deadlineFor(Index.runBackSpeedCommand()))
            .andThen(flywheel.runFullSpeedCommand()
                .alongWith(wood.WoodCommand1().withTimeout(Seconds.of(0.5)))
                .alongWith(new WaitCommand(Seconds.of(0.6))
                    .andThen(Index.runHalfSpeedCommand()))));

    controller.x().onTrue(new SwerveDriveToPointCommand(drivetrain, new Pose2d(0.6, 0, Rotation2d.fromDegrees(0))));

    controller.leftBumper().whileTrue(wood.WoodCommand1());
    controller.rightBumper().whileTrue(wood.woodLauncherCommand());

    var drivetrainCommand = new SwerveDriveTeleopCommand(drivetrain, controller);
    drivetrain.setDefaultCommand(drivetrainCommand);
    new Trigger(() -> !drivetrainCommand.isScheduled() && controller.getLeftY() > 0.3).onTrue(drivetrainCommand);

    // controller.b().onTrue(
    // flywheel.runFullSpeedCommand().withTimeout(Seconds.of(2))
    // .andThen(Index.launcherCommand().alongWith(flywheel.runFullSpeedCommand()).withTimeout(Seconds.of(2))));
    // ;

    // .andThen
    // (flywheel.runAtlaunchCommand().withTimeout(Seconds.of(2)));

    controller.rightTrigger().onTrue(new InstantCommand(drivetrain::useMegatagTwo));
    controller.leftTrigger().onTrue(new InstantCommand(drivetrain::useMegatagOne));

    drivetrain.useMegatagOne();
  }

  @Override
  public void autonomousInit() {
    new SwerveDriveToPointCommand(drivetrain, new Pose2d(15, 0.7, Rotation2d.fromDegrees(0)))
        .alongWith(
            flywheel.runBackSpeedCommand().withTimeout(Seconds.of(0.5)).deadlineFor(Index.runBackSpeedCommand()))
        .andThen(flywheel.runFullSpeedCommand()
            .alongWith(wood.WoodCommand1().withTimeout(Seconds.of(0.5)))
            .alongWith(new WaitCommand(Seconds.of(0.6))
                .andThen(Index.runHalfSpeedCommand()))
            .withTimeout(Seconds.of(5)))
        .schedule();

    drivetrain.useMegatagTwo();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
