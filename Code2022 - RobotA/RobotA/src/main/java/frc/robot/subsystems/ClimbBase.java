// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.ClimbDefault;

public class ClimbBase extends SubsystemBase {
  /** Creates a new ClimbBase. */

  private TalonFX elevateMotor; //motor that changes the elevator
  private TalonFX tipAngleMotor; //motor that tips the robot
  private DigitalInput elevatorLowerSwitch;
  private DigitalInput elevatorUpperSwitch;
  private DigitalInput angleBackSwitch;
  private DigitalInput angleFrontSwitch;

  private double tipSpeed = Constants.climbTipSpeed; //Speed for elevator to tip robot at

  public static int moving;

  public ClimbBase() {
    
    this.elevateMotor = new TalonFX(Constants.climbElevateMotor);
    this.tipAngleMotor = new TalonFX(Constants.climbTipAngleMotor);
    this.tipAngleMotor.setNeutralMode(NeutralMode.Brake);
    this.elevateMotor.setNeutralMode(NeutralMode.Brake);
    this.elevateMotor.setInverted(true);
    this.elevatorLowerSwitch = new DigitalInput(Constants.elevatorClimbLowerSwitch);
    this.elevatorUpperSwitch = new DigitalInput(Constants.elevatorClimbUpperSwitch);
    //this.angleBackSwitch = new DigitalInput(Constants.angleBackSwitch);
    //this.angleFrontSwitch = new DigitalInput(Constants.angleFrontSwitch);
    
    setDefaultCommand(new ClimbDefault(this));
    moving = 0;
    }

  //raises elevator
  public void raiseClimbElevator(){
    this.elevateMotor.set(TalonFXControlMode.PercentOutput, Constants.climbElevatorSpeedUp);
    moving = 1;
  }

  public void lowerClimbElevator(){
    this.elevateMotor.set(TalonFXControlMode.PercentOutput, Constants.climbElevatorSpeedDown);
    moving = -1;
  }

  public void zeroClimbElevator(){
    this.elevateMotor.set(TalonFXControlMode.PercentOutput, 0);
    moving = 0;
  }

  //tips robot
  public void tipForwardRobotElevator(){
    this.tipAngleMotor.set(ControlMode.PercentOutput, tipSpeed);
  }
  public void tipBackwardsRobotElevator(){
    this.tipAngleMotor.set(ControlMode.PercentOutput, -tipSpeed);
  }

  public void zeroAngleMotor(){
    this.tipAngleMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  //returns elevator motor encoder
  public double getClimbElevatorEncoder(){
    return this.elevateMotor.getSelectedSensorPosition();
  }

  //returns tip motor encoder
  public double getClimbTipAngleEncoder(){
    return this.tipAngleMotor.getSelectedSensorPosition();
  }

  public boolean getElevatorLowSwitch(){
    return !this.elevatorLowerSwitch.get();
  }
  public boolean getElevatorUpperSwitch(){
    return !this.elevatorUpperSwitch.get();
  }

  public boolean getAngleBackSwitch(){
    return !this.angleBackSwitch.get();
  }
  public boolean getAngleFrontSwitch(){
    return !this.angleFrontSwitch.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("lowSwitchClimb", getElevatorLowSwitch());
    SmartDashboard.putBoolean("upSwitchClimb", getElevatorUpperSwitch());
    
  }
}
