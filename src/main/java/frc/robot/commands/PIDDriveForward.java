// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDDriveForward extends CommandBase {
  /** Creates a new PIDDriveForward. */

  private final DriveTrain _driveTrain;
  private double distance;
  private double speed = 0;
  private final double kP = 0.5;
  private final double kI = 0.1;
  private final double kD = 0.01;
  private double errorSum = 0;
  private double errorRate = 0;
  private Timer timer;
  private double error;
  private double lastTimestamp = 0;
  private double lastError = 0;
  private double dt = 0;
  private final double IntegralLimit = 1;

  public PIDDriveForward(DriveTrain dt, double dis) {
    // Use addRequirements() here to declare subsystem dependencies.
    distance = dis;
    _driveTrain = dt;
    timer = new Timer();
    lastError = dis;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    lastTimestamp = timer.get();
    _driveTrain.resetEncoders();
    errorSum = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt = timer.get() + lastTimestamp;
    lastTimestamp = timer.get();

    error = (distance - Math.abs(_driveTrain.getPos()));
    errorRate = (error = lastError)/dt;
    
    errorRate = error;

    if(Math.abs(error) == IntegralLimit){
      errorSum += error * dt;
    }

    speed = kP * error + kI * errorSum + kD * errorRate;
    speed *= 0.15;
    _driveTrain.tankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDrive(0, 0);
    _driveTrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(_driveTrain.getPos());
    return(speed == 0);
  }
}
