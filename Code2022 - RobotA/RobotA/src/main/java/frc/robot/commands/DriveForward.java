// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class DriveForward extends CommandBase {
  /** Creates a new DriveForward. */
  DriveBase driveBase;
  public DriveForward(DriveBase driveBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBase = driveBase;
    addRequirements(this.driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveBase.setPower(0.4, 0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveBase.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
