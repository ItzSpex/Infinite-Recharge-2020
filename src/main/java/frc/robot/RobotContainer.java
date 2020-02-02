/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import java.util.Arrays;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Boomer m_shooter = new Boomer();
  private final Arm m_arm = new Arm();
  public final Intake m_intake = new Intake();
  private final Compressor m_compressor = new Compressor(1);
  /*private final Drivetrain m_robotDrive = new Drivetrain();
  private final Index m_index = new Index();
*/
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final Command m_shootCommand =
  new InstantCommand(m_shooter::enable, m_shooter).andThen(
   new WaitUntilCommand(m_shooter::atSetPoint)
  ,new InstantCommand(m_intake::TestIntake,m_intake));


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
  /*  m_robotDrive.setDefaultCommand(
            // A split-stick arcade command, with forward/backward controlled by the left
            // hand, and turning controlled by the right.
            new RunCommand(() -> m_robotDrive
                    .arcadeDrive(m_driverController.getY(GenericHID.Hand.kLeft),
                            m_driverController.getX(GenericHID.Hand.kRight)), m_robotDrive));
*/
 /* m_arm.setDefaultCommand(
          new RunCommand(() -> m_arm.moveShooter(m_operatorController.getY(GenericHID.Hand.kLeft)))
  );*/
  }


  private void configureButtonBindings() {
    //Index Button - Operator Yellow (Y) Button
//   new JoystickButton(m_operatorController, XboxController.Button.kY.value)
//        .whenPressed(new InstantCommand(m_index::moveBall, m_index))
//        .whenReleased(new InstantCommand(m_index::stopMotor, m_index));
   //Shooter Button - Operator Green (A) Button
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
            .whenPressed(m_shootCommand)
            .whenReleased(new InstantCommand(m_shooter::disable, m_shooter));
    //Start Intake - Operator Red (B) Button
    new JoystickButton(m_operatorController,XboxController.Button.kB.value)
            .whenPressed(new InstantCommand(m_intake::StartIntake, m_intake));
    //Stop Intake - Operator Blue (X) Button
    new JoystickButton(m_operatorController,XboxController.Button.kX.value)
            .whenPressed(new InstantCommand(m_intake::StopIntake, m_intake));
   new JoystickButton(m_operatorController, XboxController.Button.kBumperLeft.value)
           .whenPressed(new InstantCommand(m_arm::moveShooterDown, m_arm))
           .whenReleased(new InstantCommand(m_arm::stallShooter,m_arm));
   new JoystickButton(m_operatorController, XboxController.Button.kBumperRight.value)
           .whenPressed(new InstantCommand(m_arm::moveShooterUp, m_arm))
           .whenReleased(new InstantCommand(m_arm::stallShooter, m_arm));
   new JoystickButton(m_operatorController, XboxController.Button.kY.value)
           .whenPressed(new InstantCommand(m_arm::stopMotor,m_arm));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                            DriveConstants.kvVoltSecondsPerMeter,
                            DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics,
                    10);

    // Create config for trajectory
    TrajectoryConfig config =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(DriveConstants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
    );


    return null;
  }
}