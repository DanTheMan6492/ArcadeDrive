// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//this is a change to the github repository

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */

  private final DriveTrain _driveTrain;
  private Timer timer;

  public AutoDrive(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    timer = new Timer();

    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("STARTED");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _driveTrain.tankDrive(0.6, 0.6);
    System.out.println(timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDrive(0, 0);
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(timer.get() >= 3);
  }
}
