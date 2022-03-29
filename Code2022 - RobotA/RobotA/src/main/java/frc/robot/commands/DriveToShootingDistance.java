// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.FieldAndRobot;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimelightBase;

public class DriveToShootingDistance extends CommandBase {
  /** Creates a new DriveToShootingDistance. */
  DriveBase drivebase;
  LimelightBase limelight;

  private double distancePidP = 0.3; 
  private double distancePidI = 0; 
  private double distancePidD = 0; 
  
  private PIDController distancePidController;

  private double anglePidP = 0.05;
  private double anglePidI = 0;
  private double anglePidD = 0;

  private PIDController anglePidController;

  private double limelightAngleRadians;
  private double distanceToHub;

  public DriveToShootingDistance(DriveBase drivebase, LimelightBase limelight) {
    this.drivebase = drivebase;
    this.limelight = limelight;
    addRequirements(this.drivebase, this.limelight);

    this.distancePidController = new PIDController(this.distancePidP, this.distancePidI, this.distancePidD);
    this.distancePidController.setTolerance(0.2);
    this.distancePidController.setIntegratorRange(-0.3, 0.3);

    this.anglePidController = new PIDController(this.anglePidP, this.anglePidI, this.anglePidD);
    this.anglePidController.setTolerance(4.5, 2);
    this.anglePidController.setIntegratorRange(-0.3, 0.3);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.limelightAngleRadians = (FieldAndRobot.limelightAngleDegrees + this.limelight.getY()) * (Math.PI / 180);
    this.distanceToHub = FieldAndRobot.heightForCalculation / Math.tan(this.limelightAngleRadians);

    this.distancePidController.setSetpoint(FieldAndRobot.shootingDistance);
    this.anglePidController.setSetpoint(2);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.limelightAngleRadians = (FieldAndRobot.limelightAngleDegrees + this.limelight.getY()) * (Math.PI / 180);
    this.distanceToHub = FieldAndRobot.heightForCalculation / Math.tan(this.limelightAngleRadians);
    double powerL = this.distancePidController.calculate(this.distanceToHub) + this.anglePidController.calculate(this.limelight.getX());
    double powerR = this.distancePidController.calculate(this.distanceToHub) - this.anglePidController.calculate(this.limelight.getX());

    this.drivebase.setPower(powerL, powerR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivebase.setVelocity(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.distancePidController.atSetpoint() && this.anglePidController.atSetpoint());
  }
}
