package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final VictorSPX m_Motor =  new VictorSPX(ArmConstants.kMotorPort);

    public void moveShooter(){
        m_Motor.set(ControlMode.Velocity, ArmConstants.kArmSpeed);
    }
    public void stopMotor(){
        m_Motor.set(ControlMode.Disabled, 0);
    }
}
