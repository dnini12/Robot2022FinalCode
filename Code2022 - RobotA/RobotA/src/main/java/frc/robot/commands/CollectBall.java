// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.StorageSubsystem;

public class CollectBall extends CommandBase {
  private IntakeBase intakeBase;
  private StorageSubsystem storageSubsystem;


  public CollectBall(IntakeBase intakeBase, StorageSubsystem storageSubsystem) {
    this.intakeBase = intakeBase;
    this.storageSubsystem = storageSubsystem;

    addRequirements(this.intakeBase, this.storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.intakeBase.intakeIn();
    this.storageSubsystem.setLowStorage();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeBase.zeroIntakeMotor();
    this.storageSubsystem.zeroAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
