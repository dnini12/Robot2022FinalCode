// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LEDStrip;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  LEDStrip ledStrip;

  //public static boolean enabled
  public LEDSubsystem() {
    ledStrip = new LEDStrip(0, 40);
    
  }

  @Override
  public void periodic() {
    ledStrip.movingRGB(10, 255, 0, 255, 10, false);
    if(false){
      return;
    }
    //int delay = ((int)(10/(ShooterBase.velocity+0.001)));

    // SmartDashboard.putNumber("Math.abs(ShooterBase.velocity-ShooterBase.velocityTarget)", Math.abs(ShooterBase.velocity-ShooterBase.velocityTarget));
    // SmartDashboard.putBoolean("amit tambal",delay!=0);
    
    if (ShooterBase.velocityTarget>1||ShooterBase.velocity>1){//shooting
      if (Math.abs(ShooterBase.velocity-ShooterBase.velocityTarget)<5&&ShooterBase.velocityTarget!=0){
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
