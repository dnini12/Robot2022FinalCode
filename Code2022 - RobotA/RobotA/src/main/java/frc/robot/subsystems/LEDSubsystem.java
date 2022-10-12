// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LEDStrip;
import frc.robot.Robot;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public static boolean shoot;

  LEDStrip ledStrip;

  //public static boolean enabled
  public LEDSubsystem() {
    ledStrip = new LEDStrip(0, 40);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Differnt From Correct Speed", Math.abs(ShooterBase.velocity-ShooterBase.velocityTarget)<5&&ShooterBase.velocityTarget!=0);
    //ledStrip.movingRGB(10, 255, 0, 255, 10, false);
    if(Robot.matchStarted){
      SmartDashboard.putNumber("Timer", Timer.getMatchTime());
    }
    else{
      SmartDashboard.putNumber("Timer", -999);
    }
    
    
    if (ShooterBase.velocityTarget>1||ShooterBase.velocity>1){//shooting
      if (Math.abs(ShooterBase.velocity-ShooterBase.velocityTarget)<5&&ShooterBase.velocityTarget!=0){
        if(shoot){
          ledStrip.blinkRGB(40, 0, 255, 0);
        }
        ledStrip.staticRGB(0, 255, 0);
      }
      else if(ShooterBase.velocity>2){
        ledStrip.staticRGB(255,255,100);
      }
      else{//starting to get velocity or got to velocity
        ledStrip.staticRGB(255,255,0);
      }
    }
    else{
      if (ClimbBase.moving == 0){//just driving
        ledStrip.staticRGB(0, 109/2, 254/2);
      }
      else if (ClimbBase.moving > 0){// climbing up
        ledStrip.staticRGB(255, 0, 255);
      }
      else  if (ClimbBase.moving < 0){// climbing down
        ledStrip.staticRGB(255, 0, 0);
      }
    }
  }
}
