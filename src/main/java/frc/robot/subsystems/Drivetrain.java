package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
    //The Gyro Sensor
    AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;


    public Drivetrain()
    {
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()),m_leftEncoder.getDistance()
                         ,m_rightEncoder.getDistance());
    }

    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),m_rightEncoder.getRate());
    }

    public void resetOdometry(Pose2d pose)
    {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void arcadeDrive(double fwd, double rot)
    {
        m_drive.arcadeDrive(fwd, rot);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
    }

    public void resetEncoders()
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

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
