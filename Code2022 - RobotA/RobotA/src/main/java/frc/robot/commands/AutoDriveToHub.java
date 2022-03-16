// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimelightBase;
import frc.robot.Constants.FieldAndRobot;

public class AutoDriveToHub extends CommandBase {
  /** Creates a new AutoDriveToHub. */
  private DriveBase drivebase;
  private LimelightBase limelight;
  private double distanceToHub;
  private double limelightAngleRadians;

  private double pidP = 0.7; 
  private double pidI = 0; 
  private double pidD = 0; 
  private PIDController pidController;
  private double maxPowerDrive = 0.8;

  public AutoDriveToHub(DriveBase drivebase, LimelightBase limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebase = drivebase;
    this.limelight = limelight;
    addRequirements(this.drivebase, this.limelight);

    this.pidController = new PIDController(this.pidP, this.pidI, this.pidD);
    this.pidController.setTolerance(0.2);
    this.pidController.setIntegratorRange(-0.2, 0.2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.limelightAngleRadians = (FieldAndRobot.limelightAngleDegrees + this.limelight.getY()) * (Math.PI / 180);
    this.distanceToHub = FieldAndRobot.heightForCalculation / Math.tan(this.limelightAngleRadians);
    pidController.setSetpoint(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.limelightAngleRadians = (FieldAndRobot.limelightAngleDegrees + this.limelight.getY()) * (Math.PI / 180);
    this.distanceToHub = FieldAndRobot.heightForCalculation / Math.tan(this.limelightAngleRadians);
    double powerForward = this.pidController.calculate(this.distanceToHub)<0?Math.max(this.pidController.calculate(this.distanceToHub), -maxPowerDrive):Math.min(this.pidController.calculate(this.distanceToHub), maxPowerDrive);
    this.drivebase.setVelocity(powerForward - this.limelight.getX() * Drive.limelightRotationProportion, powerForward + this.limelight.getX() * Drive.limelightRotationProportion);
    // this.drivebase.setPower(-powerForward, -powerForward);
    //this.drivebase.setPower(this.limelight.getX() * Drive.limelightRotationProportion, -this.limelight.getX() * Drive.limelightRotationProportion);
    //SmartDashboard.putNumber("distance", this.distanceToHub);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivebase.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.pidController.atSetpoint() && (this.limelight.getX()>-2 && (this.limelight.getX()<2))||this.limelight.getA()<1) || this.limelight.getA()>40;
  }
}
