// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.StorageSubsystem;

public class SetShooterSpeed extends CommandBase {
  private ShooterBase shooterBase;
  private StorageSubsystem storageSubsystem;
  private double targetVelocity;
  /** Creates a new SetShooterSpeed. */
  public SetShooterSpeed(ShooterBase shooterBase, double targetVelocity, StorageSubsystem storageSubsystem) {
    this.shooterBase = shooterBase;
    this.targetVelocity = targetVelocity;
    this.storageSubsystem = storageSubsystem;
    addRequirements(this.shooterBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooterBase.setVelocityShooter(this.targetVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("shooter velocity now", this.shooterBase.getShooterVelocity());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterBase.zeroShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return /*OI.getXboxController().getRightBumper() ||*/ OI.getBackButton();
  }
}
