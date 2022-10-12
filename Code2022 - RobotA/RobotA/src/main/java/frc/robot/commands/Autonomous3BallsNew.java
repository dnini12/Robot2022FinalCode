// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
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

public class Autonomous3BallsNew extends SequentialCommandGroup {
  /** Creates a new Autonomous3Balls. */
  DriveBase driveBase;
  IntakeBase intakeBase;
  StorageSubsystem storageSubsystem;
  ShooterBase shooterBase;
  LimelightBase limelightBase;
  Trajectory t = new Trajectory();
  public Autonomous3BallsNew(DriveBase driveBase, IntakeBase intakeBase, StorageSubsystem storageSubsystem, ShooterBase shooterBase, LimelightBase limelightBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.driveBase = driveBase;
    this.intakeBase = intakeBase;
    this.storageSubsystem = storageSubsystem;
    this.shooterBase = shooterBase;
    this.limelightBase = limelightBase;
    
    try {
      t = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("2nd3BALLS.wpilib.json"));

    } catch (Exception e) {
      System.out.println(" Error reading json " + e);
    }
    RamseteCommand trajectoryCommand = new RamseteCommand(
      t,
      driveBase::getPose,
      new RamseteController(2, 0.7),
      driveBase.getKinematics(),
      driveBase::setVelocity,
      driveBase
    );
    

    addCommands(
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase),
      new ParallelRaceGroup(new ShootAuto(shooterBase, Constants.shootingFromHubVelocityAuto), new WaitCommand(1.3)),
      new ParallelRaceGroup(new MoveStorageAuto(storageSubsystem), new WaitCommand(1.8)),
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase),
      new InstantCommand(()->intakeBase.intakeIn(), intakeBase),
      new InstantCommand(()->storageSubsystem.setLowStorage(), storageSubsystem),
      new InstantCommand(()->driveBase.SetPose(t.getInitialPose()), driveBase),
      trajectoryCommand,

      new InstantCommand(()->driveBase.setVelocity(-1.2,-1.2), driveBase),  
      new WaitCommand(0.7),
      new InstantCommand(()->driveBase.setVelocity(0,0), driveBase),
      new Turn180Degrees(driveBase),

      new ParallelRaceGroup(new ShootAuto(shooterBase, Constants.shootingFromHubVelocityAuto),new WaitCommand(0.1)),


      new ParallelRaceGroup(new AimToHub(limelightBase, driveBase), new WaitCommand(0.7)),
      new ParallelRaceGroup(new MoveStorageBackAuto(storageSubsystem), new WaitCommand(0.4)),
      new ParallelRaceGroup(new MoveStorageAuto(storageSubsystem), new WaitCommand(0.7)),
      new ParallelRaceGroup(new StartEndCommand(()->this.storageSubsystem.setForward(),()->this.storageSubsystem.zeroAllMotors(),storageSubsystem), new WaitCommand(0.6)),
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase)
    );
  }
}
