// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
    //ledStrip.staticRGB(0, 109, 254);
    if (ShooterBase.velocityTarget>0){//shooting
      if (Math.abs(ShooterBase.velocity-ShooterBase.velocityTarget)<2){
        ledStrip.blinkRGB(((int)(ShooterBase.velocityTarget/ShooterBase.velocity))*20,0,255,0);
      }
      else if(ShooterBase.velocity>5){
        ledStrip.blinkRGB(((int)(ShooterBase.velocityTarget/ShooterBase.velocity))*20,255,255,100);
      }
      else{//starting to get velocity
        ledStrip.staticRGB(255,255,254);
      }
    }
    else{
      if (ClimbBase.moving == 0){//just driving
        ledStrip.staticRGB(0, 109, 254);
      }
      else if (ClimbBase.moving > 0){// climbing up

      }
      else  if (ClimbBase.moving < 0){// climbing down

      }
    }
  }
}
