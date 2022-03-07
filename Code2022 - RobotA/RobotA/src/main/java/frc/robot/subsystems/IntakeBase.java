// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.ChangeIntakeRotation;

public class IntakeBase extends SubsystemBase {
  /** Creates a new IntakeBase. */
  private VictorSPX angleMotor; //Motor that changes angle of intake (up or down)
  private CANSparkMax intakeMotor; //Motor that sucks balls and spits them out ;)
  

  private DigitalInput topLimitSwitch; //Limit switch is pressed when intake is raised
  private DigitalInput lowLimitSwitch; //Limit switch is pressed when intake is lowered

  private double intakeSpeed = Constants.intakeSpeed; //Speed to suck balls and spit them out ;)
  private double angleSpeedRaise = Constants.intakeAngleSpeedRaise; //Speed to raise the intake
  private double angleSpeedLower = Constants.intakeAngleSpeedLower; //Speed to lower the intake

  public IntakeBase() {
    this.angleMotor = new VictorSPX(Constants.intakeAngleMotor);
    this.intakeMotor = new CANSparkMax(Constants.intakeMotor, MotorType.kBrushless);
    this.lowLimitSwitch = new DigitalInput(Constants.lowLimitSwitch);
    this.topLimitSwitch = new DigitalInput(Constants.topLimitSwitch);
  }

  //If the intake is upright
  public boolean getTopSwitch(){
    return !this.topLimitSwitch.get();
  }

  //If the intake is down
  public boolean getLowSwitch(){
    return !this.lowLimitSwitch.get();
  }

  //Sucks the balls 
  public void intakeIn(){
    this.intakeMotor.set(this.intakeSpeed);
  }

  //Spits the balls
  public void intakeOut(){
    this.intakeMotor.set(-this.intakeSpeed);
  }

  //Raises the intake angle
  public void raiseIntake(){
    this.angleMotor.set(ControlMode.PercentOutput, -this.angleSpeedRaise);
  }

  //Lowers the intake angle
  public void lowerIntake(){
    this.angleMotor.set(ControlMode.PercentOutput, this.angleSpeedLower);
  }

  public void zeroIntakeMotor(){
    this.intakeMotor.set(0);
  }
  public void zeroAngleMotor(){
    this.angleMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    //If the intake is up or down

    //SmartDashboard.putString("Intake", (this.topLimitSwitch.get())?("Up"):("Down"));



    if (OI.getXboxController().getBButton()) {
      this.intakeIn();
    }
    else if (OI.getXboxController().getXButton()) {
      this.intakeOut();
    }
    else{
      this.zeroIntakeMotor();
    }

    if (OI.getXboxController().getLeftY()<-0.1) {
      this.raiseIntake();
    }
    else if (OI.getXboxController().getLeftY()>0.1) {
      this.lowerIntake();
    }
    else{
      this.zeroAngleMotor();
    }
    SmartDashboard.putBoolean("Upper intake", getTopSwitch());
    SmartDashboard.putBoolean("lower intake", getLowSwitch());
  }
}
