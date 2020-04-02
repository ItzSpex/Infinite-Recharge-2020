/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auton4MTime;
import frc.robot.commands.SetWristSetpoint;
import frc.robot.spikes.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.spikes.util.XboXUID;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final Boomer m_shooter = new Boomer();
  public static final Wrist m_wrist = new Wrist();
  public final Intake m_intake = new Intake();

  public static final Drivetrain m_robotDrive = new Drivetrain();
  public static final Index m_index = new Index();
  public static final Climber m_climber = new Climber();

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static XboXUID driver = new XboXUID(OIConstants.kDriverControllerPort);
  public static XboXUID operator = new XboXUID(OIConstants.kOperatorControllerPort);

  public static final Command m_shoot =
    //new InstantCommand(m_arm::moveShooterPID,m_arm).andThen(
    //new WaitUntilCommand(m_arm::atSetPoint),
  new InstantCommand(m_shooter::useOutput,m_shooter).andThen(
  new WaitUntilCommand(m_shooter::atSetPoint),
  new InstantCommand(m_index::moveInBall,m_index));

  public static final Command m_stopShooting =
 // new InstantCommand(m_arm::stopMotor).andThen(
  new InstantCommand(m_shooter::disable,m_shooter).andThen(
          new InstantCommand(m_index::stop,m_index));

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(new RunCommand(m_robotDrive::moveUsingJoystick,m_robotDrive));


  }


  private void configureButtonBindings() {
    JoystickButton shootingButton = operator.getGreenButton();
    JoystickButton useIntakeButton = operator.getRedButton();
    JoystickButton indexIn = operator.getBlueButton();
    JoystickButton stopHandStall = operator.getYellowButton();
    Button aimAndShoot = operator.getUpButton();
    JoystickButton manualWristUp = operator.getRBButton();
    JoystickButton manualWristDown = operator.getLBButton();
    Button openIntake = operator.getRightButton();
    Button closeIntake = operator.getLeftButton();
    Button reverseIndex = operator.getButtonStart();
    Button reverseClimber = operator.getButtonBack();

    Button activateMotorClimber = operator.getDownButton();
    //Shooting Button - Operator Green (A) Button
    shootingButton.whenPressed(m_shoot).whenReleased(m_stopShooting);
    //Intake - Operator Red (B) Button
    useIntakeButton.toggleWhenPressed(new MoveGenericSubsystem(m_intake, IntakeConstants.kIntakeSpeed));
    //Index - Operator Blue (X) Button
    indexIn.toggleWhenPressed(new MoveGenericSubsystem(m_index, IndexConstants.kIndexSpeed));
    //Stop hand stall- Operator Yellow (Y) Button
    stopHandStall.whenPressed(new InstantCommand(m_wrist::disable,m_wrist));
    //Aim And Shoot - Operator Up Button
    //aimAndShoot.toggleWhenPressed(new AimAndShoot(m_robotDrive,m_wrist,m_shooter));
    aimAndShoot.whenPressed(new SetWristSetpoint(m_wrist))
            .whenReleased(new InstantCommand(m_wrist::manualStall));
    //Control Shooter Down - Operator LB Button
    manualWristDown.whenPressed(new InstantCommand(m_wrist::manualDown,m_wrist))
           .whenReleased(new InstantCommand(m_wrist::manualStall,m_wrist));
    //Control Shooter Up - Operator RB Button
    manualWristUp.whenPressed(new InstantCommand(m_wrist::manualUp,m_wrist))
          .whenReleased(new InstantCommand(m_wrist::manualStall,m_wrist));
    //Open Intake - Operator Right Button
    openIntake.whenPressed(new InstantCommand(m_intake::OpenIntake,m_intake));
    //Close Intake - Operator Left Button
    closeIntake.whenPressed(new InstantCommand(m_intake::CloseIntake,m_intake));
    //Climber Motor - Operator Down Button
    activateMotorClimber.whenPressed(new InstantCommand(m_climber::ClimbUp,m_climber))
            .whenReleased(new InstantCommand(m_climber::StopMotor,m_climber));
    //Reverse Index - Operator Start Button
    reverseIndex.toggleWhenPressed(new InstantCommand(m_index::moveOutBall,m_index))
            .whenReleased(new InstantCommand(m_index::stop,m_index));
    //Reverse Climber - Operator Back Button
    reverseClimber.whenPressed(new InstantCommand(m_climber::ClimbDown,m_climber))
            .whenReleased(new InstantCommand(m_climber::StopMotor,m_climber));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Auton4MTime(m_robotDrive,m_wrist,m_shooter);
  }
}