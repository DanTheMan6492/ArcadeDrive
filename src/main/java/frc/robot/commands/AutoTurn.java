// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoTurn extends CommandBase {
  /** Creates a new AutoDriveSquare. */

  private final DriveTrain _driveTrain;
  private Timer timer;
  private String dir;

  public AutoTurn(DriveTrain dt, String direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    timer = new Timer();
    dir = direction;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(dir){
      case("right"):
        _driveTrain.tankDrive(-0.6, 0.6);
        break;
      case("left"):
        _driveTrain.tankDrive(0.6, -0.6);
        break;
      default:
        System.out.println("DIRECTION NOT VALID; DFAULTED TO RIGHT");
        _driveTrain.tankDrive(-0.6, 0.6);
        break;
    }
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
    return(timer.get() >= 0.5);
  }
}
