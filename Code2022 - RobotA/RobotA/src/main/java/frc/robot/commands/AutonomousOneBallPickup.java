// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntBinaryOperator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
public class AutonomousOneBallPickup extends SequentialCommandGroup {
  private DriveBase driveBase;
  private ShooterBase shooterBase;
  private LimelightBase limelightBase;
  private IntakeBase intakeBase;
  private StorageSubsystem storageSubsystem; //:)

  /** Creates a new AutonomousOneBallPickup. */
  public AutonomousOneBallPickup(DriveBase driveBase, ShooterBase shooterBase, LimelightBase limelightBase, IntakeBase intakeBase, StorageSubsystem storageSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.driveBase = driveBase;
    this.shooterBase = shooterBase;
    this.limelightBase = limelightBase;
    this.intakeBase = intakeBase;
    this.storageSubsystem = storageSubsystem;
    
    addCommands(
      //new ParallelRaceGroup(new ChangeIntakeRotation(this.intakeBase),new WaitCommand(1)),
      new ParallelRaceGroup(new CollectBall(this.intakeBase, this.storageSubsystem),new DriveForward(this.driveBase), new WaitCommand(1.5)),
      new ParallelRaceGroup(new CollectBall(this.intakeBase, this.storageSubsystem),new WaitCommand(0.5)),
      new ParallelRaceGroup(new CollectBall(this.intakeBase, this.storageSubsystem),new DriveForward(this.driveBase), new WaitCommand(0.1)),
      new ParallelRaceGroup(new Turn180Degrees(this.driveBase),new WaitCommand(2)),
      new ParallelRaceGroup(new StartEndCommand(()->this.storageSubsystem.setLowStorage(),()->this.storageSubsystem.zeroAllMotors()), new WaitCommand(0.4)),      
      new ParallelRaceGroup(new DriveForward(this.driveBase),new WaitCommand(1.3)),
      //new ParallelRaceGroup(new ChangeIntakeRotation(this.intakeBase),new WaitCommand(1.3)),
      new ParallelRaceGroup(new AimToHub( this.limelightBase, driveBase),new WaitCommand(2.5)),
      //new ParallelRaceGroup(new WaitCommand(1),new StartEndCommand(()->this.storageSubsystem.setBackwards(),()->this.storageSubsystem.zeroAllMotors(),this.storageSubsystem)),
      new ParallelRaceGroup(new ShootAuto(this.shooterBase, Constants.shootingFromHubVelocityAuto),new WaitCommand(2)),
      new ParallelRaceGroup(new WaitCommand(0.2),new StartEndCommand(()->this.storageSubsystem.setLowStorageBack(),()->this.storageSubsystem.zeroAllMotors())),
      new ParallelCommandGroup(new InstantCommand(()->this.storageSubsystem.setTopStorage()),new WaitCommand(2)),
      new ParallelRaceGroup(new WaitCommand(2.5),new ShootAuto(this.shooterBase, Constants.shootingFromHubVelocityAuto2),new StartEndCommand(()->this.storageSubsystem.setLowStorage(),()->this.storageSubsystem.zeroAllMotors())),
      new InstantCommand(()-> this.shooterBase.setPowerShooter(0),this.shooterBase)
      // new ParallelRaceGroup(new StartEndCommand(()-> this.driveBase.setPower(-0.45, -0.45),()-> this.driveBase.setPower(0, 0)),new WaitCommand(2.5))

      
      // new InstantCommand(()-> this.driveBase.setVelocity(-1.5, -1.5), this.driveBase).andThen(new WaitCommand(3), new InstantCommand(()-> this.driveBase.setVelocity(0, 0), this.driveBase))
      //if time left, adding command to drive back fast to get a head start
    );
  }
}
