package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class CrossLineTime extends SequentialCommandGroup {

    public CrossLineTime(Drivetrain drivetrain)
    {
        addCommands(
                new StartEndCommand(drivetrain::moveAuto,drivetrain::stopMotors,drivetrain)
                        .withTimeout(AutoConstants.kTimetopasstheline)
        );
    }
}
