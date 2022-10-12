// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.StorageSubsystem;

public class MoveStorageAuto extends CommandBase {
  /** Creates a new MoveStorageAuto. */
  StorageSubsystem storageSubsystem;
  public MoveStorageAuto(StorageSubsystem storageSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.storageSubsystem = storageSubsystem;
    addRequirements(this.storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    

    if (Math.abs(ShooterBase.velocity-ShooterBase.velocityTarget)<Constants.shooterSpeedDeadzone&&ShooterBase.velocityTarget!=0) {
      this.storageSubsystem.setForward();
    }
    else {
      this.storageSubsystem.zeroAllMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.storageSubsystem.zeroAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
