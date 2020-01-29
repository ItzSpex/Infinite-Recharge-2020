/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;

public class Index extends SubsystemBase {
  private final VictorSP m_Motor =  new VictorSP(IndexConstants.kMotorPort);

  public void moveBall(){
    m_Motor.set(IndexConstants.kIndexSpeed);
  }
  public void stopMotor(){
    m_Motor.stopMotor();
  }
}
