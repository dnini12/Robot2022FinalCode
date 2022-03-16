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
public class Autonomous3Balls extends SequentialCommandGroup {
  /** Creates a new Autonomous3Balls. */
  DriveBase driveBase;
  IntakeBase intakeBase;
  StorageSubsystem storageSubsystem;
  ShooterBase shooterBase;
  Trajectory t;
  public Autonomous3Balls(DriveBase driveBase, IntakeBase intakeBase, StorageSubsystem storageSubsystem, ShooterBase shooterBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.driveBase = driveBase;
    this.intakeBase = intakeBase;
    this.storageSubsystem = storageSubsystem;
    this.shooterBase = shooterBase;
    
    try {
      t = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("3BallPath.wpilib.json"));

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
      new ShootAuto(shooterBase, Constants.shootingFromHubVelocityAuto),
      new ParallelRaceGroup(new StartEndCommand(()->this.storageSubsystem.setTopStorage(),()->this.storageSubsystem.zeroAllMotors()), new WaitCommand(0.4)),
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase),
      new InstantCommand(()->intakeBase.intakeIn(), intakeBase),
      new InstantCommand(()->storageSubsystem.setLowStorage(), storageSubsystem),
      new InstantCommand(()->driveBase.SetPose(t.getInitialPose()), driveBase),
      trajectoryCommand,
      new ShootAuto(shooterBase, Constants.shootingFromHubVelocityAuto),
      new ParallelRaceGroup(new StartEndCommand(()->this.storageSubsystem.setTopStorage(),()->this.storageSubsystem.zeroAllMotors()), new WaitCommand(0.4)),
      new InstantCommand(()->shooterBase.setPowerShooter(0), shooterBase)
    );
  }
}
