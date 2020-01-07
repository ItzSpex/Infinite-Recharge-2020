package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final SpeedController m_Motor =
            new WPI_TalonSRX(IntakeConstants.kMotorPort);
    private final DoubleSolenoid m_leftSolenoid =
            new DoubleSolenoid(1,IntakeConstants.kLeftSolenoidPort);
    private final DoubleSolenoid m_rightSolenoid =
            new DoubleSolenoid(1,IntakeConstants.kRightSolenoidPort);
    public void OpenIntake()
    {
        m_leftSolenoid.set(DoubleSolenoid.Value.kForward);
        m_rightSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void CloseIntake()
    {
        m_leftSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public void StartIntake()
    {
        m_Motor.set(IntakeConstants.kIntakeSpeed);
    }
    public void StopIntake()
    {
        m_Motor.stopMotor();
    }
}
