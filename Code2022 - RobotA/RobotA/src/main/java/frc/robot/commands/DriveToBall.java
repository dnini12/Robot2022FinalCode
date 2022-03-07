
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimelightBase;

public class DriveToBall extends CommandBase {
  DriveBase driveBase;
  LimelightBase limelightBase;
  private double leftp;
  private double rightp;
  private double xTolerance;

  public DriveToBall(DriveBase driveBase, LimelightBase limelightBase) {
    this.limelightBase = limelightBase;
    this.driveBase = driveBase;
    this.xTolerance = 5;
    addRequirements(this.driveBase);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    this.leftp = 0;
    this.rightp = 0;
    if(this.limelightBase.getX() >= -this.xTolerance && this.limelightBase.getX() <= this.xTolerance){
      this.leftp = 0.3;
      this.rightp = 0.3;
    }
    else{
      this.leftp = (this.limelightBase.getX() > this.xTolerance)?(0.3):(-0.3);
      this.rightp = (this.limelightBase.getX() > this.xTolerance)?(-0.3):(0.3);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.driveBase.setPower(0,0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
