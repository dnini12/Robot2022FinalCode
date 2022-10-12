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

  public static boolean onTarget;
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

    // return FieldAndRobot.heightForCalculation/Math.tan(FieldAndRobot.limelightAngleDegrees+this.getY());


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("limelight distance hub", getDistanceFromHub());
    onTarget = this.getX()>-1 && this.getX()<1;
    SmartDashboard.putBoolean("On Target", onTarget);
    LEDSubsystem.shoot = onTarget;
  }
}
