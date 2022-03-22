package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.StorageDefault;

public class StorageSubsystem extends SubsystemBase {

  private CANSparkMax upperMotor;//upper motor
  private VictorSPX lowerMotor;//lower motor



  private AnalogInput distanceSenor;//distance sensor...



  public StorageSubsystem() {// constructor
    this.upperMotor = new CANSparkMax(Constants.storageUpperMotor,MotorType.kBrushless);
    this.lowerMotor = new VictorSPX(Constants.storageLowerMotor);
    
    this.distanceSenor = new AnalogInput(Constants.storageSensor);

    setDefaultCommand(new StorageDefault(this));
  }

  public void setForward(){//set power to upper and lower motor
    this.upperMotor.set(-Constants.storagePower);
    this.lowerMotor.set(ControlMode.PercentOutput, Constants.storagePower);
  }

  public void setTopStorage(){
    this.upperMotor.set(-Constants.storagePowerAuto);
  }

  public void setLowStorage(){
    this.lowerMotor.set(ControlMode.PercentOutput, Constants.storagePowerAuto);
  }
  public void setLowStorageBack(){
    this.lowerMotor.set(ControlMode.PercentOutput, -Constants.storagePowerAuto);
  }

  public void setStorageAuto(){
    this.upperMotor.set(-Constants.storagePowerAuto);
    this.lowerMotor.set(ControlMode.PercentOutput, Constants.storagePowerAuto);
  }
  
  public void setBackwards(){
    this.upperMotor.set(Constants.storagePower);
    this.lowerMotor.set(ControlMode.PercentOutput, -Constants.storagePower);
  }

  public void zeroAllMotors(){
    this.upperMotor.set(0);
    this.lowerMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getDistanceSensor(){//gets distance
    //TODO:// find out what measerment it returnts
    return this.distanceSenor.getVoltage();
  }

  public void autonomusBallDetection(){// moves the ball when it collects it to the front of the storage
    if(this.getDistanceSensor() >= Constants.minDetection && this.getDistanceSensor() <= Constants.maxDetection){
      this.setLowStorage();
    }
  }

  @Override
  public void periodic() {
    if (this.getDistanceSensor() >= Constants.minDetection+0.3 && this.getDistanceSensor() <= Constants.maxDetection){
      Constants.ballDetected = true;
    }
    SmartDashboard.putBoolean("ball detected", Constants.ballDetected);
   SmartDashboard.putNumber("storage sensor", getDistanceSensor());
   SmartDashboard.putBoolean("Storage ball in", (this.getDistanceSensor() >= Constants.minDetection && this.getDistanceSensor() <= Constants.maxDetection));
  }
}
