package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.nio.file.Path;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutonTraj extends SequentialCommandGroup {
    String trajectoryJSON = "PathWeaver/Groups/StealAndShootRed";
    Trajectory trajectory = null;
    {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    public AutonTraj()
    {
        if(trajectory != null)
        {
            addCommands(
                    new RamseteCommand(trajectory,RobotContainer.m_robotDrive::getPose,
                            new RamseteController(AutoConstants.kRamseteB,AutoConstants.kRamseteZeta),
                            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                            DriveConstants.kDriveKinematics,
                            RobotContainer.m_robotDrive::getWheelSpeeds,
                            new PIDController(DriveConstants.kPDriveVel,0,0),
                            new PIDController(DriveConstants.kPDriveVel,0,0),
                            RobotContainer.m_robotDrive::tankDriveVolts,
                            RobotContainer.m_robotDrive)
            );
        }

    }
}
