package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final WPI_VictorSPX m_Motor =
            new WPI_VictorSPX(ClimberConstants.kMotorPort);
    private final DoubleSolenoid m_Solenoid =
            new DoubleSolenoid(ClimberConstants.kSolenoidPortForward, ClimberConstants.kSolenoidPortBackward);

    public void Eject(){
        m_Solenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void Dejecet(){
        m_Solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public void ClimbUp() { m_Motor.set(ClimberConstants.kClimbSpeed); }
    public void ClimbDown() {m_Motor.set(-ClimberConstants.kClimbSpeed);}
    public void StopMotor() {
        m_Motor.stopMotor();
    }
}
