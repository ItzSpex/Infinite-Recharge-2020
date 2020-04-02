package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.spikes.command.genericsubsystem.GenericSubsystem;

public class Intake extends GenericSubsystem {
    private final SpeedController m_Motor =
            new WPI_TalonSRX(IntakeConstants.kMotorPort);
    private final DoubleSolenoid m_Solenoid =
            new DoubleSolenoid(IntakeConstants.kSolenoidPortForward,IntakeConstants.kSolenoidPortBackward);
    public void OpenIntake(){
        m_Solenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void CloseIntake(){
        m_Solenoid.set(DoubleSolenoid.Value.kReverse);
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
}
