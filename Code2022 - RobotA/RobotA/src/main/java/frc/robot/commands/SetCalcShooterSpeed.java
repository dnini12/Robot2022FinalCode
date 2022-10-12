// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.FieldAndRobot;
import frc.robot.subsystems.LimelightBase;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.StorageSubsystem;

public class SetCalcShooterSpeed extends CommandBase {
  private ShooterBase shooterBase;
  private StorageSubsystem storageSubsystem;
  private double targetVelocity;

  private LimelightBase limelightBase;
  /** Creates a new SetShooterSpeed. */
  public SetCalcShooterSpeed(ShooterBase shooterBase, StorageSubsystem storageSubsystem, LimelightBase limelightBase) {
    this.shooterBase = shooterBase;
    this.storageSubsystem = storageSubsystem;
    this.limelightBase = limelightBase;
    addRequirements(this.shooterBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("shooter velocity now", this.shooterBase.getShooterVelocity());
    SmartDashboard.putNumber("Distance from hub", calcDistance(limelightBase));

    targetVelocity = calcVelocity(calcDistance(limelightBase));

    this.shooterBase.setVelocityShooter(targetVelocity<29?targetVelocity:29);
    // this.shooterBase.setVelocityShooter(2);
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


  static double calcDistance(LimelightBase limelightBase) {
    double limelightAngleRadians = (FieldAndRobot.limelightAngleDegrees +limelightBase.getY()) * (Math.PI / 180);
    return FieldAndRobot.heightForCalculation / Math.tan(limelightAngleRadians);
  }

  static double calcVelocity(double distance) {
    return 8.1067*Math.pow(Constants.e, 0.4083*distance) ;
  }
}
