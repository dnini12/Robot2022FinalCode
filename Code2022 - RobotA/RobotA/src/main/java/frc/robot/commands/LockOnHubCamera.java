// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimelightBase;

public class LockOnHubCamera extends CommandBase {
  private LimelightBase limelightBase;
  private DriveBase driveBase;

  private double p = 0;
  
  private double pidP = 0;
  private double pidI = 0;
  private double pidD = 0;

  private double maxPowerDrive = 0.7;

  private PIDController pidHeading;
  
  public LockOnHubCamera(LimelightBase limelightBase, DriveBase driveBase) {
    this.driveBase = driveBase;
    this.limelightBase = limelightBase;
    addRequirements(driveBase);

    this.pidHeading = new PIDController(this.pidP, this.pidI, this.pidD);

    this.pidHeading.setTolerance(2);
    this.pidHeading.setIntegratorRange(-0.2, 0.2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pidHeading.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    p = pidHeading.calculate(limelightBase.getX()<0?Math.max(this.pidHeading.calculate(this.limelightBase.getX()), -maxPowerDrive):Math.min(this.pidHeading.calculate(this.limelightBase.getX()), maxPowerDrive));

    driveBase.setPower(p, -p);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidHeading.atSetpoint();
  }
}
