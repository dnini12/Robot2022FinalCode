// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static final double e = Math.exp(1);

    //intakeAngleLowerSpeed, shootingMotor, shootingAngle, shootingAngleSpeed
    //Intake
    public static final int intakeAngleMotor = 13; //Connection for motor that changes the intake's angle
    public static final int intakeMotor = 21; //Connection for the motor that turns ball-swallower ;)
    public static final int topLimitSwitch = 1; //Connection for imit switch pressed when intake is upright
    public static final int lowLimitSwitch = 0; //Connection for imit switch pressed when intake is down
    public static final double intakeSpeed = 1; //Speed to swallow balls with ;)
    public static final double intakeAngleSpeedRaise = 0.6; //Speed to raise intake with
    public static final double intakeAngleSpeedLower = 0.35; //Speed to lower intake with


    
    //Shooter
    public static final int shootingMotor = 12; //Connection for motor that turns the ball-shooting wheels
    public static final int shootingAngle = 16; //Connection for motor that changes the shooter's angle
    public static final double shootingChangeAngleSpeed = 0.3;//Speed (percentage) to change shooter's angle with
    public static final double shooterUp = 0;//encoder value to shoot from hub
    public static final double shooterDown = 0;// encoder value to shoot from hub

    public static final double shootingSpeedDeadzone = 0.0;

    public static final double shootingFromHubVelocity = 18.5;
    public static final double shootingFromHubVelocityAuto = 18.5;
    public static final double shootingFromHubVelocityAuto2 = 19;
    public static final double shootingFromLaunchpadVelocity = 0.0;

    public static final double shooterPulseToMeter = 0.4788/2048;

    //Climb
    public static final int climbElevateMotor = 16; //Connection for motor that lifts the elevator
    public static final int climbTipAngleMotor = 17; //Connection for motor that tips the robot while hanging
    public static final double climbElevatorSpeedUp = -1; //Speed for which elevator goes up at
    public static final double climbElevatorSpeedDown = 1;
    public static final double climbTipSpeed = 1; //Speed for which angle to tip at
    public static final double climbElevatorEncoderToMeter = 0;//How many ticks for a meter(Elevator)
    public static final double climbAngleEncoderToAngle = 0;//How many ticks for 1(Angle)
    public static final int elevatorClimbLowerSwitch = 3;
    public static final int elevatorClimbUpperSwitch = 2;
    public static final int angleBackSwitch = 99;
    public static final int angleFrontSwitch = 99;

    //Storage
    public static final double shooterSpeedDeadzone = 0.8;
    public static final double storagePower = 0.7;//power that the storage system motors use
    public static final double storagePowerAuto = 0.65;
    public static final double maxDetection = 3;//max measurment when we detect the ball
    public static final double minDetection = 1.2;//min measurment when we detect the ball
    public static final int storageLowerMotor = 15;
    public static final int storageUpperMotor = 20;
    public static final int storageSensor = 0;
    public static boolean ballDetected;

    //OI
    public static final int leftJoystickPort = 1;
    public static final int rightJoystickPort = 0;
    public static final int xboxControllerPort = 2;
    public static final double OIDeadzone = 0.175;
    

    

    //DriveBase
    public static class Drive {
    public static final int driveLeftUpperMotor = 14;
    public static final int driveLeftLowerMotor = 10;
    public static final int driveRightUpperMotor = 19;
    public static final int driveRightLowerMotor = 18;
    public static final double limelightRotationProportion = 0.03;

    public static final double drivebaseEncoderPerMeter = 45700;
    public static final double maxPower = 0.6;

    public static final double drivePulseToMeter = (double)1/45700;
    }

    //Field & Robot
    public static class FieldAndRobot{
        public static final double upperHubHeight = 2.64; //in meters
        public static final double limelightAngleDegrees = 30;
        public static final double limelightHeight = 0.81;
        public static final double heightForCalculation = upperHubHeight - limelightHeight;
        public static final double leftoverDist = 0; //distance from when 'AutoDriveToHub' ends to target distance from hub
        public static final double shootingDistance = 0;
    }

}