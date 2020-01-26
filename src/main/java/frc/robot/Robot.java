/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;


  /*
   NetworkTable is a collection of databases stored in the RoboRIO
   The co processor can change data inside it
   This database can be viewed in a program named OutlineViewer
   "table" represents a single database called "MyCamName" under "Chameleon Vision`"
   */
  NetworkTable table;

  /*
  targetX represents the horizontal angle
  targetY represents the vertical angle
  */
  NetworkTableEntry targetX;
  NetworkTableEntry targetY;



  //Error values for the control loop
  double rotationError;
  double distanceError;

  //Control loop constants
    /*
        This example uses proportional control loop with constant force
        After you master proportional control use might want to try PID control loop
    */
  double KpRot=-0.1;
  double KpDist=-0.1;

  //Deadzone is necessary because the robot can only get so accurate and cannot be pefectly head on the target
  double angleTolerance=5;//Deadzone for the angle control loop
  double distanceTolerance=5;//Deadzone for the distance control loop

  /*
  There is a minimum power that you need to give to the drivetrain in order to overcome friction
  It helps the robot move and rotate at low speeds
  */
  double constantForce=0.05;

  /*
  rotationAjust is rotational signal for the drivetrain
  distanceAjust is forward signal for the drivetrain
  */
  double rotationAjust;
  double distanceAjust;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //Points "table" to the NetworkTable database called "chameleon-vision"
    table=NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("MyCamName");

    //Points to the database value named "yaw" and "pitch"
    targetX=table.getEntry("yaw");
    targetY=table.getEntry("pitch");
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    rotationAjust=0;
    distanceAjust=0;
    if (m_robotContainer.m_operatorController.getAButtonPressed())//the "A" button
    {
            /*
                Fetches the rotation and distance values from the vision co processor
                sets the value to 0.0 if the value doesnt exist in the database
            */
      rotationError=targetX.getDouble(0.0);
      distanceError=targetY.getDouble(0.0);

            /*
                Proportional (to targetX) control loop for rotation
                Deadzone of angleTolerance
                Constant power is added to the direction the control loop wants to turn (to overcome friction)
            */
      if(rotationError>angleTolerance)
        rotationAjust=KpRot*rotationError+constantForce;
      else
      if(rotationError<angleTolerance)
        rotationAjust=KpRot*rotationError-constantForce;
            /*
                Proportional (to targetY) control loop for distance
                Deadzone of distanceTolerance
                Constant power is added to the direction the control loop wants to turn (to overcome friction)
            */
      if(distanceError>distanceTolerance)
        distanceAjust=KpDist*distanceError+constantForce;
      else
      if(distanceError<distanceTolerance)
        distanceAjust=KpDist*distanceError-constantForce;


      //Output the power signals to a arcade drivetrain
      m_robotContainer.m_robotDrive.arcadeDrive(distanceAjust,rotationAjust);
    }
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
