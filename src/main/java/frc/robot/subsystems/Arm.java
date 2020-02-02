package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final SpeedController m_Motor = new WPI_TalonSRX(ArmConstants.kMotorPort);
    public void moveShooterDown() {m_Motor.set(ArmConstants.kArmDown);}
    public void moveShooterUp(){ m_Motor.set(ArmConstants.kArmUp);}
    public void stallShooter() {m_Motor.set(ArmConstants.kArmStall);}
    public void stopMotor(){
        m_Motor.stopMotor();
    }
}
