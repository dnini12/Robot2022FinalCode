// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.ClimbBase;

public class ClimbDefault extends CommandBase {
  private ClimbBase climbBase;


  
  public ClimbDefault(ClimbBase climbBase) {
    this.climbBase = climbBase;
    addRequirements(this.climbBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(OI.getXboxController().getPOV()==0&&!this.climbBase.getElevatorUpperSwitch()){
      this.climbBase.raiseClimbElevator();
    }
    else if(OI.getXboxController().getPOV()==180&&!this.climbBase.getElevatorLowSwitch()){
      this.climbBase.lowerClimbElevator();
    }
    else{
      this.climbBase.zeroClimbElevator();
    }

    if(OI.getXboxController().getPOV()==90){
      this.climbBase.tipForwardRobotElevator();
      
    }
    else if(OI.getXboxController().getPOV()==270){
      this.climbBase.tipBackwardsRobotElevator();
      
    }
    else{
      this.climbBase.zeroAngleMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
