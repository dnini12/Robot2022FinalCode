// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous3BallsAndPickup extends SequentialCommandGroup {
  /** Creates a new Autonomous3Balls. */
  DriveBase driveBase;
  IntakeBase intakeBase;
  StorageSubsystem storageSubsystem;
  ShooterBase shooterBase;
  Trajectory threeBalls = new Trajectory();
  Trajectory ballPickup = new Trajectory();

  public Autonomous3BallsAndPickup(DriveBase driveBase, IntakeBase intakeBase, StorageSubsystem storageSubsystem, ShooterBase shooterBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.driveBase = driveBase;
    this.intakeBase = intakeBase;
    this.storageSubsystem = storageSubsystem;
    this.shooterBase = shooterBase;
    
    try {
      threeBalls = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("3BallPath.wpilib.json"));
      ballPickup = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("Ball3to4,path.wpilib.json"));


    } catch (Exception e) {
      System.out.println(" Error reading json " + e);
    }
    RamseteCommand threeBallsCommand = new RamseteCommand(
      threeBalls,
      driveBase::getPose,
      new RamseteController(2, 0.7),
      driveBase.getKinematics(),
      driveBase::setVelocity,
      driveBase
    );

    RamseteCommand ballPickupCommand = new RamseteCommand(
      ballPickup,
      driveBase::getPose,
      new RamseteController(2, 0.7),
      driveBase.getKinematics(),
      driveBase::setVelocity,
      driveBase
    );
    

    addCommands(
      new ParallelRaceGroup(new ShootAuto(shooterBase, Constants.shootingFromHubVelocityAuto), new WaitCommand(1.5)),
      new ParallelRaceGroup(new StartEndCommand(()->this.storageSubsystem.setTopStorage(),()->this.storageSubsystem.zeroAllMotors()), new WaitCommand(0.4)),
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase),
      new ParallelRaceGroup(new ChangeIntakeRotation(intakeBase), new WaitCommand(1)),
      new InstantCommand(()->intakeBase.intakeIn(), intakeBase),
      new InstantCommand(()->storageSubsystem.setLowStorage(), storageSubsystem),
      new InstantCommand(()->driveBase.SetPose(threeBalls.getInitialPose()), driveBase),
      threeBallsCommand,
      new InstantCommand(()->intakeBase.zeroIntakeMotor(), intakeBase),
      new ParallelRaceGroup(new ShootAuto(shooterBase, Constants.shootingFromHubVelocityAuto), new WaitCommand(2.5)),
      new ParallelRaceGroup(new StartEndCommand(()->this.storageSubsystem.setBackwards(),()->this.storageSubsystem.zeroAllMotors()), new WaitCommand(0.3)),
      new ParallelRaceGroup(new StartEndCommand(()->this.storageSubsystem.setForward(),()->this.storageSubsystem.zeroAllMotors()), new WaitCommand(1.2)),
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase),
      new InstantCommand(()->intakeBase.intakeIn(), intakeBase),
      new InstantCommand(()->storageSubsystem.setLowStorage(), storageSubsystem),
      ballPickupCommand,
      new WaitCommand(1)
    );
  }
}
