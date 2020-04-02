package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Boomer;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Wrist;

public class AimAndShoot extends SequentialCommandGroup {
    public AimAndShoot(Wrist wrist, Boomer shooter) {
        addCommands(
        new SetWristSetpoint(wrist),
        parallel(
        new InstantCommand(wrist::manualStall)),
        RobotContainer.m_shoot);
    }

}
