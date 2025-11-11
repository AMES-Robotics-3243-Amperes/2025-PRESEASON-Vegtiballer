// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveDriveTeleopCommand extends Command {

  private final SwerveDrivetrain swerveDrivetrain;
  private final CommandXboxController controller;
  /** Creates a new SwerveDriveTeleopCommand. */
  public SwerveDriveTeleopCommand(SwerveDrivetrain swerveDrivetrain, CommandXboxController controller) {
    this.swerveDrivetrain = swerveDrivetrain;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = controller.getLeftY();
    double ySpeed = controller.getLeftX();
    double angularSpeed = controller.getRightX();
    swerveDrivetrain.drive(xSpeed, ySpeed, angularSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
