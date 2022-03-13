// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterBase extends SubsystemBase {
  /** Creates a new ShooterBase. */
  private TalonFX shootingWheels; //the wheels that shoot the ball(s) out of the shooter
  private VictorSPX shootingAngle; //The motor that changes the angle of the shooter
  private Encoder angleEncoder;//shooting angle encoder

  private double angleMotorPercentage = Constants.shootingChangeAngleSpeed; //percentage power to change angle with (default)

  private SimpleMotorFeedforward feedforward;
  //feedforeward values
  private double ks = 0.02519;
  private double kv = 0.021433;
  //pid values
  private double pidP = 0.01;
  private double pidI = 0;
  private double pidD = 0;

  public static double velocity;
  public static double velocityTarget;


  public ShooterBase() {
    this.shootingWheels = new TalonFX(Constants.shootingMotor);
    //this.shootingAngle = new VictorSPX(Constants.shootingAngle);
    this.shootingWheels.config_kP(0, this.pidP);
    this.shootingWheels.config_kI(0, this.pidI);
    this.shootingWheels.config_kD(0, this.pidD);

    this.feedforward = new SimpleMotorFeedforward(ks, kv);

  }

  //Shoot out balls by percentage
  public void setPowerShooter(double p){
    this.shootingWheels.set(ControlMode.PercentOutput, p);
  }

  //Shoot out balls by speed
  public void setVelocityShooter(double v){
    this.shootingWheels.set(TalonFXControlMode.Velocity, v/Constants.shooterPulseToMeter/10, DemandType.ArbitraryFeedForward,feedforward.calculate(v));  
    //SmartDashboard.putNumber("Velocity target",  v/Constants.shooterPulseToMeter/10);
    velocityTarget = v;
    //SmartDashboard.putNumber("shooter velocity now", getShooterVelocity());
  }

  //Changes angle of shooter up
  public void changeShooterAngleUp(){
    this.shootingAngle.set(ControlMode.PercentOutput, this.angleMotorPercentage);
  }

  //Changes angle of shooter down
  public void changeShooterAngleDown(){
    this.shootingAngle.set(ControlMode.PercentOutput, -this.angleMotorPercentage);
  }

  public void zeroAngleMotor(){
    this.shootingAngle.set(ControlMode.PercentOutput, 0);
  }
  //Zeroing the motors
  public void zeroShooterMotors(){
    velocityTarget = 0;
    this.shootingWheels.set(TalonFXControlMode.PercentOutput, 0);
  }

  //Gets encoder for the shooting wheels
  public double getShootingWheelsEncoder(){
    return this.shootingWheels.getSelectedSensorPosition();
  }

  public double getShooterVelocity(){
    return this.shootingWheels.getSelectedSensorVelocity()*Constants.shooterPulseToMeter*10;
  }
  
 public double getShooterEncoder(){// get shootingwheels encoder pulse
  return this.shootingWheels.getSelectedSensorPosition()*Constants.shooterPulseToMeter*10;
}

public double getAngleEncoder(){//get angle encoder value
  return this.angleEncoder.getDistance();
}

public void print(){
  SmartDashboard.putNumber("shooter velocity", getShooterVelocity());
}
  @Override
  public void periodic() {
    velocity = getShooterVelocity();
    SmartDashboard.putNumber("shooter velocity", getShooterVelocity());
    //SmartDashboard.putNumber("shooter velocity target", velocityTarget);
  }
}
