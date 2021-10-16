// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private  AHRS navx = new AHRS(SPI.Port.kMXP);
  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _righttDriveTalon;

  private final int TPR = 4096;
  private final double Circum = 1.0;

  private DifferentialDrive _diffDrive;


  /** Creates a new DriveTrain. */
  public DriveTrain() {

    
    _leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    _righttDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

    _leftDriveTalon.configFactoryDefault();
    _leftDriveTalon.setInverted(false);
    _leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    _righttDriveTalon.setInverted(false);

    _diffDrive = new DifferentialDrive(_leftDriveTalon, _righttDriveTalon);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double xSlide, double ySlide){
    _diffDrive.arcadeDrive(ySlide, xSlide);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    _diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public double getPos(){
    return(_leftDriveTalon.getSelectedSensorPosition(0) / TPR * Circum);
  }

  public double getVel(){
    return(_leftDriveTalon.getSensorCollection().getPulseWidthVelocity() * TPR * Circum);
  }

  public void resetEncoders(){
    _leftDriveTalon.setSelectedSensorPosition(0, 0, 10);
  }

  public double getNavAngle(){
    return(navx.getAngle());
  }

  public void resetNavX(){
    navx.reset();
  }
}
