// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTurn extends CommandBase {
  /** Creates a new DriveTurn. */

  private final DriveTrain _driveTrain;
  private int dir1;
  private double angle;

  public DriveTurn(DriveTrain dt, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    dir1 = (int) ((int) Math.abs(angle) / angle);
    this.angle = angle;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _driveTrain.resetNavX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _driveTrain.tankDrive(-1 * 0.6 * dir1,  0.6 * dir1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(Math.abs(_driveTrain.getNavAngle()) > Math.abs(angle));
  }
}
