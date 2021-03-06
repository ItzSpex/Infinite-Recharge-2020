/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int kPCMPort = 0;

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 5;
        public static final int kLeftMotor2Port = 6;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;

        public static final int[] kLeftEncoderPorts = new int[]{4, 5};
        public static final int[] kRightEncoderPorts = new int[]{6, 7};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = true;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 1.55;
        public static final double kvVoltSecondsPerMeter = 3.0;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0325;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.557;
    }
    public static final class BoomerConstants {
        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
        public static final int kEncoderCPR = 2048;
        public static final double kEncoderDistancePerPulse =
                1.0 / (double) kEncoderCPR;

        public static final int kLeftMotorPort = 1;
        public static final int kRightMotorPort = 2;

        public static final double kShooterFreeRPS = 5300;
        public static final double kShooterTargetRPS = 4000;
        public static final double kShooterToleranceRPS = 50;

        public static final double kShooterSpeed = 0.8;
        public static final double kSVolts = 1.5;
        public static final double kVVoltSecondPerRotation = 12.0/kShooterFreeRPS;

        public static final double kLeftP = 0.0254;
        public static final double kLeftI = 0;
        public static final double kLeftD = 0.001;

        public static final double kRightP = 0.0254;
        public static final double kRightI = 0;
        public static final double kRightD = 0.001;

        public static final double minMotorValue = 0.1;
        public static final double maxMotorValue = 0.9;

    }
    public static final class IntakeConstants {
        public static final int kMotorPort = 7;
        public static final int kSolenoidPortForward = 2;
        public static final int kSolenoidPortBackward = 3;
        public static final double kIntakeSpeed = -0.8;
    }
    public static final class IndexConstants {
        public static final int kMotorPort = 8;
        public static final double kIndexSpeed = 0.8;
    }

    public static final class WristConstants {
        public static final int kMotorPort = 9;
        public static final int kPotentiometerPort = 0;

        public static final double kUpSpeed = -0.7;
        public static final double kDownSpeed = 0.05;
        public static final double kStallSpeed = -0.2;

        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kTolerance = 1;
        public static final double kSetpoint = 36;
        public static final double kOffsetFunc = 40;

        public static final double kFullRangeDegrees = 90;
        public static final double kOffsetDegrees = 0;

    }
public static final class ClimberConstants {
        public static final int kMotorPort = 10;
        public static final int kSolenoidPortForward = 0;
        public static final int kSolenoidPortBackward = 1;
        public static final double kClimbSpeed = -0.9;
}
public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
}

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kBackwardspeed = 0.8;
        public static final double kTurnSpeed = 0;
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kTimetopasstheline = 0.7;
    }
    public static final class VisionConstants {
        public static final double kAngleTolerance = 0.5;
        public static final double kConstantForce = 0.05;
        public static final double kPRot = 0.1;
    }
}