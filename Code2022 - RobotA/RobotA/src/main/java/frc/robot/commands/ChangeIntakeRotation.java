// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.IntakeBase;

public class ChangeIntakeRotation extends CommandBase {
  /** Creates a new ChangeIntakeRotation. */
  private IntakeBase intakeBase;
  private boolean up;
  public ChangeIntakeRotation(IntakeBase intakeBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeBase = intakeBase;
    addRequirements(this.intakeBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.up = this.intakeBase.getTopSwitch()?true:false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    if(this.up){
      if(!this.intakeBase.getLowSwitch()){
        this.intakeBase.lowerIntake();
      }
      else{
        this.intakeBase.zeroAngleMotor();
      }
    }
    else{
      if(!this.intakeBase.getTopSwitch()){
        this.intakeBase.raiseIntake();
      }
      else{
        this.intakeBase.zeroAngleMotor();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeBase.zeroAngleMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean endCondition = this.up?this.intakeBase.getLowSwitch():this.intakeBase.getTopSwitch();
    return endCondition || OI.getBackButton();
  }
}
