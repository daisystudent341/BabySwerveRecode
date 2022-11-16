// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax motorTurn;
  private TalonFX motorDrive;
  private CANCoder encoder;
  double angleOffset;
  PIDController drivePID = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
  ProfiledPIDController turnPID = new ProfiledPIDController(Constants.kpTurn, Constants.kiTurn, Constants.kdTurn, new TrapezoidProfile.Constraints(Constants.maxTurnVelocity, Constants.maxTurnAcceleration));

  /** Creates a new SwerveModule. 
   * @param encoderPort */
  public SwerveModule(int drivePort, int turnPort, int encoderPort, double angleOffset) { 
    motorTurn = new CANSparkMax(turnPort,MotorType.kBrushless);
    motorDrive = new TalonFX(drivePort);
    encoder = new CANCoder(encoderPort);
    this.angleOffset = angleOffset;
    
    
}
  public double getSpeed() {

    return motorDrive.getSelectedSensorVelocity() * Constants.VELOCITY_CONVERSION_FACTOR;

  }
  //inverting state so it goes in correct dir.
  public void setDriveInverted(boolean state){
    motorDrive.setInverted(state);

  }

  public void setTurnInverted(boolean state){
    motorTurn.setInverted(state);

  }

  public Rotation2d getCurrentAngle(){
  
    double currentAngle = encoder.getAbsolutePosition() - angleOffset;
    return Rotation2d.fromDegrees(currentAngle);

  }


  public void setDesiredState(SwerveModuleState desiredState){
    
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getCurrentAngle());

    double driveOutput = drivePID.calculate(getSpeed(), optimizedState.speedMetersPerSecond);
    double turnOutput = turnPID.calculate(getCurrentAngle().getRadians(),optimizedState.angle.getRadians());
    motorDrive.set(TalonFXControlMode.PercentOutput, driveOutput);
    motorTurn.set(turnOutput);
  }







  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
