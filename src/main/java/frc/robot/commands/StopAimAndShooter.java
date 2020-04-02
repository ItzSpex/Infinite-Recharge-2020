package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Boomer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;

public class StopAimAndShooter extends SequentialCommandGroup {
    public StopAimAndShooter(Wrist wrist, Boomer shooter)
    {
        addCommands(
                new InstantCommand(wrist::disable),
                new InstantCommand(shooter::disable)
        );
    }
}
