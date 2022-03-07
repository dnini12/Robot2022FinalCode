// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveBase;

public class DrivePerMeterAuto extends CommandBase {
  /** Creates a new DrivePerMeterAuto. */
  private DriveBase drivebase;
  //numbers for pid
  private double distancePIDp;
  private double distancePIDi;
  private double distancePIDd;
  
  private double anglePIDp;
  private double anglePIDi;
  private double anglePIDd;

  //setpoint by angle, new pid
  private double distanceSetPoint;
  private PIDController distancePID;

  private double angleSetpoint;
  private PIDController anglePID;

  //tolerance and range
  private double distanceTolerance = 0.1;
  private double distanceMaxRange = 0.5;
  
  private double angleTolerance = 1;
  private double angleMaxRange = 0.4;

  private double distanceMetersError;
  
  private double lp;
  private double rp;
  public DrivePerMeterAuto(DriveBase drivebase, double meters) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebase = drivebase;

    this.distancePIDp = 0.5;
    this.distancePIDi = 0;
    this.distancePIDd = 0;

    this.distancePID = new PIDController(this.distancePIDp, this.distancePIDi, this.distancePIDd);
    this.distancePID.isContinuousInputEnabled();
    this.distancePID.setTolerance(this.distanceTolerance);
    this.distancePID.setIntegratorRange(-distanceMaxRange, distanceMaxRange);

    this.anglePIDp = 0.05;
    this.anglePIDi = 0;
    this.anglePIDd = 0;

    this.anglePID = new PIDController(this.anglePIDp, this.anglePIDi, this.anglePIDd);
    this.anglePID.isContinuousInputEnabled();
    this.anglePID.setTolerance(this.angleTolerance);
    this.anglePID.setIntegratorRange(-angleMaxRange, angleMaxRange);

    this.distanceMetersError = meters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.distanceMetersError += this.drivebase.getEncoderValueLeft() / Drive.drivebaseEncoderPerMeter;
    this.distancePID.setSetpoint(this.distanceMetersError);
    this.anglePID.setSetpoint(this.drivebase.getGyroYaw());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.lp  = this.distancePID.calculate(this.drivebase.getEncoderValueLeft() / Drive.drivebaseEncoderPerMeter);
    this.lp = (this.lp > 0)?(Math.min(this.angleMaxRange, this.lp)):(Math.max(-this.angleMaxRange, this.lp));
    this.drivebase.setPower(1, 1);
    
    System.out.println("left " + this.drivebase.getEncoderValueLeft());
    //System.out.println("SetPoint" + this.distancePID.getSetpoint());
    System.out.println("Error" + this.drivebase.getEncoderValueLeft() / Drive.drivebaseEncoderPerMeter);
    System.out.println("Power" + this.lp);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivebase.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return this.distancePID.atSetpoint() && this.anglePID.atSetpoint();
    return false;
  }
}
