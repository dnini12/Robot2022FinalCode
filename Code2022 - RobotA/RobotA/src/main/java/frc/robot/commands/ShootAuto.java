// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterBase;

public class ShootAuto extends CommandBase {
  private ShooterBase shooterBase;
  private double targetVelocity;
  /** Creates a new SetShooterSpeed. */
  public ShootAuto(ShooterBase shooterBase, double targetVelocity) {
    this.shooterBase = shooterBase;
    this.targetVelocity = targetVelocity;
    addRequirements(this.shooterBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterBase.setVelocityShooter(this.targetVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //this.shooterBase.zeroShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
