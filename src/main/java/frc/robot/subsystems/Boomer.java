package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BoomerConstants;

public class Boomer extends SubsystemBase {
    private final SpeedController m_leftMotor = new WPI_VictorSPX(BoomerConstants.kLeftMotorPort);
    private final SpeedController m_rightMotor = new WPI_VictorSPX(BoomerConstants.kRightMotorPort);

    private final Encoder m_leftEncoder = new Encoder(
     BoomerConstants.kLeftEncoderPorts[0]
    ,BoomerConstants.kLeftEncoderPorts[1]
    ,BoomerConstants.kLeftEncoderReversed);

    private final Encoder m_rightEncoder = new Encoder(
    BoomerConstants.kRightEncoderPorts[0]
    ,BoomerConstants.kRightEncoderPorts[1]
    ,BoomerConstants.kRightEncoderReversed);

    private final SimpleMotorFeedforward m_Feedforward =
            new SimpleMotorFeedforward(BoomerConstants.kSVolts
            ,BoomerConstants.kVVoltSecondPerRotation);

    private PIDController m_leftController ,m_rightController;

    public Boomer() {
        m_leftController = new PIDController(BoomerConstants.kLeftP,BoomerConstants.kLeftI,BoomerConstants.kLeftD);
        m_rightController = new PIDController(BoomerConstants.kRightP,BoomerConstants.kRightI,BoomerConstants.kRightD);
        m_leftEncoder.setDistancePerPulse(BoomerConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(BoomerConstants.kEncoderDistancePerPulse);
        resetEncoders();
    }
    public void resetEncoders()
    {
        m_rightEncoder.reset();
        m_leftEncoder.reset();
    }
    public void useOutput(){
        enable();
        m_leftMotor.set(getMeasurement() + m_Feedforward.calculate(BoomerConstants.kShooterSpeed));
        m_rightMotor.set(-(getMeasurement() + m_Feedforward.calculate(BoomerConstants.kShooterSpeed)));
        //m_rightMotor.set(BoomerConstants.kShooterSpeed);
        //m_leftMotor.set(BoomerConstants.kShooterSpeed);
    }
    public double getMeasurement() {
        return (m_leftEncoder.getRate() + m_rightEncoder.getRate())/2.0;
    }
    public boolean atSetPoint() {
        return m_rightController.atSetpoint() && m_leftController.atSetpoint();
    }
    public void disable() {
        m_rightMotor.stopMotor();
        m_leftMotor.stopMotor();
        m_leftController.disableContinuousInput();
        m_leftController.disableContinuousInput();
    }
    public void enable() {
        m_leftController.enableContinuousInput(BoomerConstants.minMotorValue,BoomerConstants.maxMotorValue);
        m_rightController.enableContinuousInput(BoomerConstants.minMotorValue,BoomerConstants.maxMotorValue);
    }
    public double getLeftSpeed(){
        return m_leftMotor.get();
    }
    public double getRightSpeed(){
        return m_rightMotor.get();
    }
}
