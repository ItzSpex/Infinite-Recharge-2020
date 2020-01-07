/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse = (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
                // Assumes the encoders are directly mounted on the wheel shafts

    }
    public static final class BoomerConstants {
        public static final int kLeftMotorPort = 0;
        public static final int kRightMotorPort = 1;
        public static final double kShootingSpeed = 0.7;
    }
    public static final class AutoConstants {
        public static final double kAutoDriveDistanceInches = 60;
        public static final double kAutoBackupDistanceInches = 20;
        public static final double kAutoDriveSpeed = 0.5;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

}
