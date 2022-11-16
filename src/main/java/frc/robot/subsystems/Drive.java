// Copyright (c) FIRST and other WPILib contribuhttps://software-metadata.revrobotics.com/REVLib.jsontors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
 SwerveModule frontLeft;
 SwerveModule frontRight;
 SwerveModule backLeft;
 SwerveModule backRight;
 SwerveDriveKinematics kinematics;
 SwerveDriveOdometry odometry;
 AHRS gyro;
  
  //AnalogInput mTurnEncoder;

  public Drive() { //this is a constructer
    frontLeft = new SwerveModule(Constants.FRONT_LEFT_DRIVE, Constants.FRONT_LEFT_TURN, Constants.FRONT_LEFT_TURN_ENCODER, Constants.FRONT_LEFT_ANGLE_OFFSET);
    frontRight = new SwerveModule(Constants.FRONT_RIGHT_DRIVE, Constants.FRONT_RIGHT_TURN, Constants.FRONT_RIGHT_TURN_ENCODER, Constants.FRONT_RIGHT_ANGLE_OFFSET);
    backLeft = new SwerveModule(Constants.BACK_LEFT_DRIVE, Constants.BACK_LEFT_TURN, Constants.BACK_LEFT_TURN_ENCODER, Constants.BACK_LEFT_ANGLE_OFFSET);
    backRight = new SwerveModule(Constants.BACK_RIGHT_DRIVE, Constants.BACK_RIGHT_TURN, Constants.BACK_RIGHT_TURN_ENCODER, Constants.BACK_RIGHT_ANGLE_OFFSET);
    kinematics = new SwerveDriveKinematics(new Translation2d(Constants.MODULE_DISTANCE, Constants.MODULE_DISTANCE ), new Translation2d(Constants.MODULE_DISTANCE, -Constants.MODULE_DISTANCE ), new Translation2d(-Constants.MODULE_DISTANCE, Constants.MODULE_DISTANCE ), new Translation2d(-Constants.MODULE_DISTANCE, -Constants.MODULE_DISTANCE ));
    gyro = new AHRS(SPI.Port.kMXP);
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

    //mTurnEncoder = new AnalogInput(0);
  }
 
  public void setStates(ChassisSpeeds Speeds) {
    SwerveModuleState [] swerveStateArray = kinematics.toSwerveModuleStates(Speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveStateArray, Constants.MAX_SPEED_MS);
    frontLeft.setDesiredState(swerveStateArray[0]);
    frontRight.setDesiredState(swerveStateArray[1]);
    backLeft.setDesiredState(swerveStateArray[2]);
    backRight.setDesiredState(swerveStateArray[3]);
    
  }
    
  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
