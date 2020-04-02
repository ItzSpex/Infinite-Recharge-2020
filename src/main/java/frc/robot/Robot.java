/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.VisionConstants;

import static frc.robot.RobotContainer.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Compressor m_compressor;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public NetworkTableEntry yaw;
  public NetworkTableEntry pitch;
  public NetworkTableEntry isDriverMode;
  public NetworkTableInstance table;
  public NetworkTable cameraTable;
  JoystickButton driverMode;
  JoystickButton aimToTarget;
  Button armPointA;
  Button armPointB;
  Button armPointC;
  Button stopCompressor;
  Button startCompressor;
  public static double currPitch;
  double rotationError = 0, toTurn = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    m_compressor = new Compressor(Constants.kPCMPort);
    m_compressor.stop();
    m_robotContainer = new RobotContainer();




    table = NetworkTableInstance.getDefault();

   cameraTable = table.getTable("chameleon-vision").getSubTable("LifeCam");

    yaw = cameraTable.getEntry("targetYaw");
    pitch = cameraTable.getEntry("targetPitch");

    isDriverMode = cameraTable.getEntry("driverMode");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
     driverMode = driver.getGreenButton();
     aimToTarget = driver.getRedButton();
     startCompressor = operator.getLeftStickButton();
     stopCompressor = operator.getRightStickButton();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    toTurn = 0;
    if(driverMode.get())
    {
      isDriverMode.setBoolean(true);
    }
    else
    {
      isDriverMode.setBoolean(false);
    }

    if(aimToTarget.get())
    {
      rotationError = yaw.getDouble(0.0);
      System.out.println("rotation Error: " + rotationError);
      if(rotationError > VisionConstants.kAngleTolerance)
      {
        toTurn = VisionConstants.kPRot * rotationError + VisionConstants.kConstantForce;
      }
      else if (rotationError < VisionConstants.kAngleTolerance)
      {
        toTurn = VisionConstants.kPRot * rotationError + VisionConstants.kConstantForce;
      }
      m_robotDrive.arcadeDrive(0.05,toTurn);
    }
    if(startCompressor.get())
      m_compressor.start();
    if(stopCompressor.get())
      m_compressor.stop();
    if(operator.getRTAxis() > 0)
      m_climber.Eject();
    if(operator.getLTAxis() > 0)
      m_climber.Dejecet();
  }



  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
