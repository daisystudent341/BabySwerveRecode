// Copyright (c) FIRST and other WPILib contribuhttps://software-metadata.revrobotics.com/REVLib.jsontors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
 
  CANSparkMax motorTurn;
  CANSparkMax motorDrive;
  //AnalogInput mTurnEncoder;

  public Drive() {
    motorTurn = new CANSparkMax(0,MotorType.kBrushless);
    motorDrive = new CANSparkMax(0,MotorType.kBrushless);

    //mTurnEncoder = new AnalogInput(0);
  }
  public void voltage() {
    //if(==true){
    motorDrive.setVoltage(3.0);
   // }
    
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
