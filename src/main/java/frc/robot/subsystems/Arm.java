package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final SpeedController m_Motor = new WPI_TalonSRX(ArmConstants.kMotorPort);
    private final AnalogPotentiometer m_potentiometer = new AnalogPotentiometer(ArmConstants.kPotPort);
    public Arm()
    {
    }
    public void moveShooterDown() {
        m_Motor.set(ArmConstants.kArmDown);
    }
    public void moveShooterUp(){
        m_Motor.set(ArmConstants.kArmUp);
    }
    public void moveShooterWithVision(){
        double turnValue = ArmConstants.kSetPoint - m_potentiometer.get() + ArmConstants.kExtraVoltage;
        System.out.println("Pot Value: " + m_potentiometer.get());
        System.out.println("turnValue: " + turnValue);
        m_Motor.setVoltage(turnValue);
    }
    public void stallShooter() {m_Motor.set(ArmConstants.kArmStall);}
    public void stopMotor(){
        m_Motor.stopMotor();
    }


}
