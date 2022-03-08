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
  LEDStrip ledStripLeft;
  LEDStrip ledStripRight;
  public LEDSubsystem() {
    ledStripLeft = new LEDStrip(0, 40);
    
  }

  @Override
  public void periodic() {
    ledStripLeft.staticRGB(0, 109, 254);
    
  }
}
