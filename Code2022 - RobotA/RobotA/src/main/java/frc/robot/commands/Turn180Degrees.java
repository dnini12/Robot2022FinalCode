// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class Turn180Degrees extends CommandBase {

  private DriveBase driveBase;

  //numbers for pid
  private double pidP;
  private double pidI;
  private double pidD;

  //setpoint by angle, new pid
  private double setPoint;
  private PIDController TurnPID;

  //TODO error to ignore, max range for power to motors
  private double tolerance = 1;
  private double maxRange = 0.4;

  public Turn180Degrees(DriveBase driveBase) {

    this.driveBase = driveBase;
    addRequirements(this.driveBase);

    this.pidP = 0.033;
    this.pidI = 0;
    this.pidD = 0.001;

    this.TurnPID = new PIDController(this.pidP, this.pidI, this.pidD);
   this.TurnPID.isContinuousInputEnabled();
    this.TurnPID.setTolerance(this.tolerance,2);
    this.TurnPID.setIntegratorRange(-maxRange, maxRange);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.setPoint = (this.driveBase.getGyroYaw() > 0) ? (this.driveBase.getGyroYaw() - 180) : (this.driveBase.getGyroYaw() + 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("setpoint",this.setPoint);
    double power = this.TurnPID.calculate(this.driveBase.getGyroYaw(),this.setPoint)>0?Math.min(this.TurnPID.calculate(this.driveBase.getGyroYaw(),this.setPoint), this.maxRange):Math.max(this.TurnPID.calculate(this.driveBase.getGyroYaw(),this.setPoint), -this.maxRange);
    this.driveBase.setPower(-power, power);
    //SmartDashboard.putNumber("power",power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveBase.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.TurnPID.atSetpoint();
  }
}
