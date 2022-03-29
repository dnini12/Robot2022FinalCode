// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.IntakeBase;

public class IntakeDfault extends CommandBase {
  /** Creates a new IntakeDrfault. */

  IntakeBase intakeBase;
  public IntakeDfault(IntakeBase intakeBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeBase = intakeBase;
    addRequirements(intakeBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.getXboxController().getBButton()) {
      intakeBase.intakeIn();
    }
    else if (OI.getXboxController().getXButton()) {
      intakeBase.intakeOut();
    }
    else{
      intakeBase.zeroIntakeMotor();
    }

    //  if (OI.getXboxController().getLeftY()<-0.1) {
    //   intakeBase.raiseIntake();
    //  }
    //  else if (OI.getXboxController().getLeftY()>0.1) {
    //   intakeBase.lowerIntake();
    //  }
    //  else{
    //   intakeBase.zeroAngleMotor();
    //  }
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
