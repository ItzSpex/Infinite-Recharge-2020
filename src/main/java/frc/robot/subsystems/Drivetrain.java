package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

    private final SpeedControllerGroup m_leftMotors =
            new SpeedControllerGroup(new VictorSP(DriveConstants.kLeftMotor1Port),
                                     new VictorSP(DriveConstants.kLeftMotor2Port));

    private final SpeedControllerGroup m_rightMotors =
            new SpeedControllerGroup(new VictorSP(DriveConstants.kRightMotor1Port),
                                     new VictorSP(DriveConstants.kRightMotor2Port));

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors,m_rightMotors);

    private final Encoder m_leftEncoder =
            new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
                    DriveConstants.kLeftEncoderReversed);

    private final Encoder m_rightEncoder =
            new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
                    DriveConstants.kRightEncoderReversed);

    public Drivetrain()
    {
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    }

    public void arcadeDrive(double fwd, double rot)
    {
        m_drive.arcadeDrive(fwd, rot);
    }

    public void resetEncoder()
    {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    }

    public Encoder getLeftEncoder() {
        return m_leftEncoder;
    }

    public Encoder getRightEncoder() {
        return m_rightEncoder;
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
}
