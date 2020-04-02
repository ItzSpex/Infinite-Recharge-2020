/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;
import frc.robot.spikes.command.genericsubsystem.GenericSubsystem;

public class Index extends GenericSubsystem {
  private final WPI_TalonSRX m_Motor = new WPI_TalonSRX(IndexConstants.kMotorPort);

  public Index() {
    m_Motor.setInverted(true);
  }

  @Override
  public void apply(double speed) {
    m_Motor.set(speed);
  }

  @Override
  public boolean canMove(double speed) {
    return true;
  }

  @Override
  public void stop() {
    m_Motor.stopMotor();
  }

  public void moveInBall(){
    m_Motor.set(IndexConstants.kIndexSpeed);
  }

  public void moveOutBall() {m_Motor.set(-IndexConstants.kIndexSpeed);}

}
