package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FlywheelJoysticCommand;
import frc.robot.commands.SwerveDriveTeleopCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.WoodSubsystem;

public class Robot extends TimedRobot {
//  WoodSubsystem wood = new WoodSubsystem();
 FlywheelSubsystem flywheel = new FlywheelSubsystem();
 IndexSubsystem Index = new IndexSubsystem();
 CommandXboxController controller = new CommandXboxController(0);
 FlywheelJoysticCommand flywheelJoysticCommand = new FlywheelJoysticCommand(controller, flywheel);
 SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  public Robot() {
   /*flywheel.setDefaultCommand(flywheelJoysticCommand);
   Trigger leftx = new Trigger(() -> controller.getLeftX()<0);
    controller.a().and(leftx).onTrue(
    flywheel.runFullSpeedCommand()
   );/* */
    // flywheel.setDefaultCommand(flywheel.runIdleSpeedCommand());

   controller.a().whileTrue(
    Index.runHalfSpeedCommand().alongWith(flywheel.runBackSpeedCommand()).withTimeout(Seconds.of(2)));
   //controller.x().whileTrue(Index.runBackSpeedCommand()).withTimeout(Seconds.of(3));
   controller.b().onTrue(Index.runBackSpeedCommand()
   .alongWith(flywheel.runBackSpeedCommand())
   .withTimeout(Seconds.of(0.8))
   .andThen(flywheel.runFullSpeedCommand()
    .alongWith(new WaitCommand(Seconds.of(0.8))
    .andThen(Index.runHalfSpeedCommand()))
    .withTimeout(Seconds.of(5))));

    // controller.x().whileTrue(wood.WoodCommand1().withTimeout(Seconds.of(1)));
    // controller.y().whileTrue(wood.woodLauncherCommand().withTimeout(Seconds.of(3)));

    drivetrain.setDefaultCommand(new SwerveDriveTeleopCommand(drivetrain, controller));

  //  controller.b().onTrue(
  //   flywheel.runFullSpeedCommand().withTimeout(Seconds.of(2))
  //   .andThen(Index.launcherCommand().alongWith(flywheel.runFullSpeedCommand()).withTimeout(Seconds.of(2))));
  // ;

    //.andThen
    //(flywheel.runAtlaunchCommand().withTimeout(Seconds.of(2)));
  }
  @Override
  public void autonomousInit() {
   flywheel.runFullSpeedCommand().withTimeout(Seconds.of(3))
    .andThen(new WaitCommand(Seconds.of(3)))
    .andThen(flywheel.runBackSpeedCommand().withTimeout(Seconds.of(3))).schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
