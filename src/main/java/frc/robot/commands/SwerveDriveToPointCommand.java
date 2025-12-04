// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveDriveToPointCommand extends Command {
  private final SwerveDrivetrain drivetrain;

  private final PIDController xController = new PIDController(2, 0.01, 0);
  private final PIDController yController = new PIDController(2, 0.01, 0);
  private final PIDController thetaController = new PIDController(4, 0, 0);

  /** Creates a new SwerveDriveToPointCommand. */
  public SwerveDriveToPointCommand(SwerveDrivetrain drivetrain, Pose2d target) {
    this.drivetrain = drivetrain;
    
    xController.setSetpoint(target.getX());
    xController.setTolerance(0.04);
    xController.setIntegratorRange(-1, 1);

    yController.setSetpoint(target.getY());
    yController.setTolerance(0.04);
    yController.setIntegratorRange(-1, 1);

    thetaController.setSetpoint(MathUtil.angleModulus(target.getRotation().getRadians()));
    thetaController.setTolerance(0.2);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPosition = drivetrain.getPosition();
    
    drivetrain.drive(
      -xController.calculate(currentPosition.getX()), 
      -yController.calculate(currentPosition.getY()),
      -thetaController.calculate(MathUtil.angleModulus(currentPosition.getRotation().getRadians())),
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
