package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BoomerConstants;

public class Boomer extends SubsystemBase {
    private final SpeedController m_leftMotor =
            new WPI_TalonSRX(BoomerConstants.kLeftMotorPort);
    private final SpeedController m_rightMotor =
            new WPI_TalonSRX(BoomerConstants.kRightMotorPort);

    public void shootBall(){
        m_rightMotor.setInverted(true);
        m_leftMotor.set(BoomerConstants.kShootingSpeed);
        m_rightMotor.set(BoomerConstants.kShootingSpeed);
    }
    public void StopMotors()
    {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

}
