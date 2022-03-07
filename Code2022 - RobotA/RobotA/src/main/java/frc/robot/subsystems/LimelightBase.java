// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldAndRobot;

public class LimelightBase extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;

  public LimelightBase() {
    tx = table.getEntry("tx");//x axis
    ty = table.getEntry("ty");//y axis
    ta = table.getEntry("ta");//how much % the target is of the screen
    tv = table.getEntry("tv");//1 if it sees a target else 0
  }

  public double getY(){//get Y
    return ty.getDouble(0.0);
  }

  public double getX(){//get X
    return tx.getDouble(0.0);
  }

  public double getA(){//get A
    return ta.getDouble(0.0);
  }

  public double getV(){//get V
    return tv.getDouble(0.0);
  }

  public double getDistanceFromHub(){
    double limelightAngleRadians = (FieldAndRobot.limelightAngleDegrees +getY()) * (Math.PI / 180);
    return FieldAndRobot.heightForCalculation / Math.tan(limelightAngleRadians);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("tx", getX());
    SmartDashboard.putNumber("ty", getY());
    SmartDashboard.putNumber("ta", getA());
    SmartDashboard.putNumber("tv", getV());
    SmartDashboard.putNumber("limelight distance hub", getDistanceFromHub());
  }
}