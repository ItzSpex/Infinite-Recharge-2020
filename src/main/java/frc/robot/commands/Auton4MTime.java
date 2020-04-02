package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Boomer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;

public class Auton4MTime extends SequentialCommandGroup {
    public Auton4MTime(Drivetrain drivetrain, Wrist wrist, Boomer shooter)
    {
        addCommands(
                new AimAndShoot(wrist,shooter),
                new WaitCommand(5),
                new StopAimAndShooter(wrist,shooter),
                new CrossLineTime(drivetrain)
        );
    }
}
