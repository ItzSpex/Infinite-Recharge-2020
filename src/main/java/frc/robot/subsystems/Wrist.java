package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;

public class Wrist extends PIDSubsystem {
    private final WPI_TalonSRX m_motor;
    private final AnalogPotentiometer m_pot;

    /**
     * Create a new wrist subsystem.
     */
    public Wrist() {
        super(new PIDController(WristConstants.kP,WristConstants.kI,WristConstants.kD));
        getController().setTolerance(WristConstants.kTolerance);

        m_motor = new WPI_TalonSRX(WristConstants.kMotorPort);

        // Conversion value of potentiometer varies between the real world and
        m_pot = new AnalogPotentiometer(WristConstants.kPotentiometerPort,WristConstants.kFullRangeDegrees,WristConstants.kOffsetDegrees);
        addChild("Wrist Motor", m_motor);
        addChild("Wrist Pot",m_pot);
    }

    // Let's name everything on the LiveWindow

    /**
     * The log method puts interesting information to the SmartDashboard.
     */
    public void log() {
        System.out.println("Wrist Angle: " + m_pot.get());
        System.out.println("Curr setpoint: " + m_controller.getSetpoint());
        System.out.println("output: " + m_motor.get());
    }

    /**
     * Use the potentiometer as the PID sensor. This method is automatically called by the subsystem.
     */
    @Override
    public double getMeasurement() {
        return m_pot.get();
    }

    /**
     * Use the motor as the PID output. This method is automatically called by the subsystem.
     */
    @Override
    public void useOutput(double output, double setpoint) {
        m_motor.set(output);
        m_controller.setSetpoint(setpoint);
    }
    public double getOutput()
    {
        return m_controller.calculate(getMeasurement(),getSetpointByVision(Robot.currPitch));
    }
    /**
     * Call log method every loop.
     */
    @Override
    public void periodic() {
        log();
    }
    public void manualUp() {
        m_motor.set(WristConstants.kUpSpeed);
    }
    public void manualStall() {
        m_motor.set(WristConstants.kStallSpeed);
    }
    public void manualDown() {
        m_motor.set(WristConstants.kDownSpeed);
    }
    public double getSetpointByVision(double pitch) {
        double angle = WristConstants.kSetpoint;
        if (pitch != 0)
        {
            angle = pitch * 2 + WristConstants.kOffsetFunc;
        }
        m_controller.setSetpoint(angle);
        return angle;
    }
}
