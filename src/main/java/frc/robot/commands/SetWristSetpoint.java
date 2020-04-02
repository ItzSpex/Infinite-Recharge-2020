package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Wrist;

/**
 * Move the wrist to a given angle. This command finishes when it is within the tolerance, but
 * leaves the PID loop running to maintain the position. Other commands using the wrist should make
 * sure they disable PID!
 */
public class SetWristSetpoint extends CommandBase {
    private final Wrist m_wrist;
    private final double m_setpoint;

    /**
     * Create a new SetWristSetpoint command.
     *
     * @param wrist    The wrist to use
     */
    public SetWristSetpoint(Wrist wrist) {
        m_wrist = wrist;
        m_setpoint = m_wrist.getSetpointByVision(Robot.currPitch);
        addRequirements(m_wrist);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_wrist.enable();
        m_wrist.setSetpoint(m_setpoint);
    }

    @Override
    public void execute() {
        m_wrist.useOutput(m_wrist.getOutput(),m_setpoint);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished()
    {
        return m_wrist.getController().atSetpoint();
    }
}