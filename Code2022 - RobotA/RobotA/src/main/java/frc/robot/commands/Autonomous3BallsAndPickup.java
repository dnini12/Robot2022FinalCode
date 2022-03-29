// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Array;
import java.util.Arrays;

import javax.naming.PartialResultException;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.LimelightBase;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous3BallsAndPickup extends SequentialCommandGroup {
  /** Creates a new Autonomous3Balls. */
  DriveBase driveBase; // Adding Var for drive System
  IntakeBase intakeBase; // Adding Var for intake System
  StorageSubsystem storageSubsystem; // Adding Var for storage System
  ShooterBase shooterBase; // Adding Var for shooter System
  LimelightBase limelightBase;
  Trajectory threeBallsTrajectory = new Trajectory(); // Creates the trajectory for the first part of the Autonomous
  Trajectory ballPickupTrajectory = new Trajectory(); // Creates the trajectory for the second part of the Autonomous
  //Trajectory ballPickupManualTrajectory = new Trajectory();

  public Autonomous3BallsAndPickup(DriveBase driveBase, IntakeBase intakeBase, StorageSubsystem storageSubsystem, ShooterBase shooterBase, LimelightBase limelightBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.driveBase = driveBase;
    this.intakeBase = intakeBase;
    this.storageSubsystem = storageSubsystem;
    this.shooterBase = shooterBase;
    this.limelightBase = limelightBase;
    try {
      threeBallsTrajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("3BallPath.wpilib.json"));
      ballPickupTrajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("Ball3to4,path.wpilib.json"));
    } catch (Exception e) {
      System.out.println(" Error reading json " + e);
    }

    RamseteCommand threeBallsCommand = new RamseteCommand( // Creates the command for the first path
      threeBallsTrajectory,
      driveBase::getPose,
      new RamseteController(2, 0.7),
      driveBase.getKinematics(),
      driveBase::setVelocity,
      driveBase
    );

    RamseteCommand ballPickupCommand = new RamseteCommand( // Creates the command for the second path
      ballPickupTrajectory,
      driveBase::getPose,
      new RamseteController(2, 0.7),
      driveBase.getKinematics(),
      driveBase::setVelocity,
      driveBase
    );


    Constants.ballDetected = false;
    addCommands(
      new ParallelRaceGroup(new ShootAuto(shooterBase, Constants.shootingFromHubVelocityAuto), new WaitCommand(1.3)),// Lowers Intake
      new ParallelRaceGroup(new StartEndCommand(()->this.storageSubsystem.setTopStorage(),()->this.storageSubsystem.zeroAllMotors()),new InstantCommand(()->intakeBase.intakeIn(), intakeBase), new WaitCommand(1.5)),
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase), // Stops shooter
      new InstantCommand(()->storageSubsystem.setLowStorage(), storageSubsystem), // Starts to move lower storage
      new InstantCommand(()->driveBase.SetPose(threeBallsTrajectory.getInitialPose()), driveBase), // Sets the robots position on the map 
      new ParallelRaceGroup(threeBallsCommand,// Path to 2+3 balls
      new ShootAuto(shooterBase, Constants.shootingFromHubVelocityAuto)), // Charges up the shooter        
      
      new ParallelRaceGroup(new AimToHubAuto(limelightBase, driveBase), new InstantCommand(()->intakeBase.zeroIntakeMotor(), intakeBase), new WaitCommand(0.7)),// Stops collecting balls
      new ParallelRaceGroup(new MoveStorageBackIf(storageSubsystem), new WaitCommand(0.4)),
      new ParallelRaceGroup(new MoveStorageIf(storageSubsystem), new WaitCommand(0.7)),

      new InstantCommand(() -> this.storageSubsystem.zeroAllMotors()),// Stops motors
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase), // Stops powering shooter
      new InstantCommand(()->intakeBase.intakeIn(), intakeBase), // Starts collecting balls
      new InstantCommand(()->storageSubsystem.setLowStorage(), storageSubsystem), // Starts lower storage
      ballPickupCommand // Takes path to 4th ball
    );
  }
}
