package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private PIDController m_potpidController = new PIDController(Constants.ArmConstants.kP,Constants.ArmConstants.kI,Constants.ArmConstants.kD);
    private AnalogInput m_potentiometer = new AnalogInput(Constants.ArmConstants.kPotPort);;

    public Arm()
    {
        m_potpidController.setSetpoint(ArmConstants.kSetPoint);
        m_potpidController.enableContinuousInput(ArmConstants.kMinValue, ArmConstants.kMaxValue);
    }
    private final SpeedController m_Motor = new WPI_TalonSRX(ArmConstants.kMotorPort);
    public void moveShooterDown() {m_Motor.set(ArmConstants.kArmDown);}
    public void moveShooterUp(){ m_Motor.set(ArmConstants.kArmUp);}
    public void stallShooter() {m_Motor.set(ArmConstants.kArmStall);}
    public void stopMotor(){
        m_Motor.stopMotor();
    }

    public void moveShooterPID()
    {
        m_Motor.set(m_potpidController.calculate(m_potentiometer.getAverageVoltage()));
    }
    public boolean atSetPoint()
    {
        return m_potpidController.atSetpoint();
    }
}
