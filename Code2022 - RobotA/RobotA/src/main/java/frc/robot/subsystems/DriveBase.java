package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;

import frc.robot.OI;
import frc.robot.commands.InstantCommmandInDisable;
import frc.robot.commands.TeleopDrive;


public class DriveBase extends SubsystemBase {


  //feedforawrd values 
  //double ks = 0.275448;
  //double kv = 0.0186;

  //double ks = 0.073;
  //double kv = 0.333;

  //double ks = 0.05033;
  //double kv = 0.238;

  double kv = 0.2;
  double ks = 0.04;

  //pid values
  double pidP = 0.0001;
  double pidI = 0;
  double pidD = 0;

  SimpleMotorFeedforward feedForward; // feedforward...
  //motors
  // private TalonFX rightFront;// front right motor
  // private TalonFX rightBack;// right back motor
  // private TalonFX leftFront;// front left motor
  // private TalonFX leftBack;// left back motor

  private TalonFX[] left;
  private TalonFX[] right;

  private TalonFX[] initMotors(int m1, int m2, boolean inverted) {
    TalonFX[] res = new TalonFX[2];
    res[0] = new TalonFX(m1);
    res[1] = new TalonFX(m2);
    //res[1].follow(res[0]);
    for(TalonFX t : res) {
      if(t!=res[0]){
        t.follow(res[0]);
      }
      t.setInverted(inverted);
      t.config_kP(0, pidP);
      t.config_kI(0, pidI);
      t.config_kD(0, pidD);
      t.setNeutralMode(NeutralMode.Coast);
      t.setSelectedSensorPosition(0);
    }
    return res;
  }

  private AHRS gyro;//gyro
  
  DifferentialDriveOdometry odometry;
  Field2d field2d = new Field2d();
  DifferentialDriveKinematics kinematics;

  public DriveBase() {//constructor
    this.gyro = new AHRS();
    this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroYaw()));
    kinematics = new DifferentialDriveKinematics(0.56);
    SmartDashboard.putData("Field",field2d);


    this.feedForward = new SimpleMotorFeedforward(ks, kv);

    this.left = initMotors(Drive.driveLeftLowerMotor, Drive.driveLeftUpperMotor,false);
    this.right = initMotors(Drive.driveRightLowerMotor, Drive.driveRightUpperMotor,true);
    SetPose(7.65, 1.8, Rotation2d.fromDegrees(0));
    setDefaultCommand(new TeleopDrive(this));
    SmartDashboard.putData("Brake",new InstantCommmandInDisable(this::setBreak,this));
    SmartDashboard.putData("Coast",new InstantCommmandInDisable(this::setCoast,this));

  
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      getLeftVelocity(), 
      getRightVelocity()
      );
  }

  public void SetOutputVolts(double leftV,double rightV){
    left[0].set(ControlMode.PercentOutput, leftV/12);
    right[0].set(ControlMode.PercentOutput, rightV/12);
  }

  public void SetPose(double x, double y, Rotation2d heading){
    left[0].setSelectedSensorPosition(0);
    right[0].setSelectedSensorPosition(0);
    odometry.resetPosition(new Pose2d(x, y, heading), Rotation2d.fromDegrees(getGyroYaw()));
  }
  public void SetPose(Pose2d pose){
    left[0].setSelectedSensorPosition(0);
    right[0].setSelectedSensorPosition(0);
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroYaw()));
  }
  
  public void setPower(double powerL,double powerR){//set power to all motors
    this.left[0].set(ControlMode.PercentOutput, powerL*Drive.maxPower);
    this.right[0].set(ControlMode.PercentOutput, powerR*Drive.maxPower);
  }

  public void setCoast() {
    left[0].setNeutralMode(NeutralMode.Coast);
    left[1].setNeutralMode(NeutralMode.Coast);
    right[0].setNeutralMode(NeutralMode.Coast);
    right[1].setNeutralMode(NeutralMode.Coast);
  }
  public void setBreak() {
    left[0].setNeutralMode(NeutralMode.Brake);
    left[1].setNeutralMode(NeutralMode.Brake);
    right[0].setNeutralMode(NeutralMode.Brake);
    right[1].setNeutralMode(NeutralMode.Brake);
  }

  public void setVelocity(double VelocityL,double VelocityR){// set velocity to all motors
    //TODO://find out what yehidat mida it gives
    this.left[0].set(TalonFXControlMode.Velocity, VelocityL/Drive.drivePulseToMeter/10, DemandType.ArbitraryFeedForward,feedForward.calculate(VelocityL));
    this.right[0].set(TalonFXControlMode.Velocity, VelocityR/Drive.drivePulseToMeter/10, DemandType.ArbitraryFeedForward,feedForward.calculate(VelocityR));
    SmartDashboard.putNumber("FF", feedForward.calculate(VelocityR));
    SmartDashboard.putNumber("Velocity target",  VelocityR/Drive.drivePulseToMeter/10);
  }

 public double getGyroYaw(){//get gyro X axis
   return -this.gyro.getYaw();
 }


 public double getGyroRoll(){//get gyro Z axis
   return this.gyro.getRoll();
 }


 public double getGyroPitch() {//get gyro Y axis
   return (double)this.gyro.getPitch();
 }

 public double getEncoderValueRight(){// get right encoder pulse
  return this.right[0].getSelectedSensorPosition();
 }

 public double getEncoderValueLeft(){// get left encoder pulse
   return this.left[0].getSelectedSensorPosition();
 }

 public double getRightDistance(){
   return this.getEncoderValueRight()*Drive.drivePulseToMeter;
 }
 public double getLeftDistance(){
  return this.getEncoderValueLeft()*Drive.drivePulseToMeter;
 }

 public double getLeftVelocity(){
   return this.left[0].getSelectedSensorVelocity()*Drive.drivePulseToMeter*10;
 }
 public double getRightVelocity(){
  return this.right[0].getSelectedSensorVelocity()*Drive.drivePulseToMeter*10;
}

public void print(){
  SmartDashboard.putNumber("left velocity", getLeftVelocity());
  SmartDashboard.putNumber("right velocity", getRightVelocity());
  SmartDashboard.putNumber("FF", feedForward.calculate(1));
    SmartDashboard.putNumber("Velocity target",  1/Drive.drivePulseToMeter/10);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("GyroYaw", getGyroYaw());
    SmartDashboard.putNumber("GyroRoll", getGyroRoll());
    SmartDashboard.putNumber("GyroPitch", getGyroPitch());
    // SmartDashboard.putNumber("RightEncoder", getEncoderValueRight());
    // SmartDashboard.putNumber("LeftEncoder", getEncoderValueLeft());
    // SmartDashboard.putNumber("Left velocity", getLeftVelocity());
    // SmartDashboard.putNumber("Right velocity", getRightVelocity());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());
    //SmartDashboard.putNumber("FF error", right[0].getClose8dLoopError());
    //SmartDashboard.putNumber("FF target", right[0].getClosedLoopTarget());
    //SmartDashboard.putNumber("power", right[0].getMotorOutputPercent());
    odometry.update(Rotation2d.fromDegrees(getGyroYaw()), getLeftDistance(), getRightDistance());
    field2d.setRobotPose(odometry.getPoseMeters());
  }
}
