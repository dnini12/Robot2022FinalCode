// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.LimelightBase;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTaxiPickUp extends SequentialCommandGroup {
  /** Creates a new AutonomousTaxiPickUp. */
  private DriveBase driveBase;
  private ShooterBase shooterBase;
  private LimelightBase limelightBase;
  private IntakeBase intakeBase;
  private StorageSubsystem storageSubsystem; //:)
  
  public AutonomousTaxiPickUp(DriveBase driveBase, ShooterBase shooterBase, LimelightBase limelightBase, IntakeBase intakeBase, StorageSubsystem storageSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.driveBase = driveBase;
    this.shooterBase = shooterBase;
    this.limelightBase = limelightBase;
    this.intakeBase = intakeBase;
    this.storageSubsystem = storageSubsystem;
    addCommands(
      new ParallelRaceGroup(new CollectBall(this.intakeBase, this.storageSubsystem),new DriveForward(this.driveBase), new WaitCommand(1.5)),
      new ParallelRaceGroup(new CollectBall(this.intakeBase, this.storageSubsystem),new WaitCommand(0.5)),
      new ParallelRaceGroup(new CollectBall(this.intakeBase, this.storageSubsystem),new DriveForward(this.driveBase), new WaitCommand(0.1))
    );
  }
}
