// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoDriveToHub;
import frc.robot.commands.AutonomousOneBallPickup;
import frc.robot.commands.CollectBall;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DrivePerMeterAuto;
import frc.robot.commands.Turn180Degrees;
import frc.robot.subsystems.ClimbBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.LimelightBase;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.StorageSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  

  public final DriveBase driveBase = new DriveBase();
  public final IntakeBase intakeBase = new IntakeBase();
  public final ShooterBase shooterBase = new ShooterBase();
  public final StorageSubsystem storageSubsystem = new StorageSubsystem();
  public final ClimbBase climbBase = new ClimbBase();
  public final OI robot_oi = new OI(storageSubsystem, intakeBase, driveBase, shooterBase, climbBase);
  public final LimelightBase limelightBase = new LimelightBase();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new InstantCommand(()-> driveBase.setPower(0.3,0.3), driveBase).andThen(new WaitCommand(1), new InstantCommand(()-> driveBase.print() , driveBase),
    //return new AutonomousOneBallPickup(this.driveBase, this.shooterBase, this.limelightBase, this.intakeBase, this.storageSubsystem);
    //return new AutoDriveToHub(this.driveBase, this.limelightBase);
    //return new CollectBall(intakeBase, storageSubsystem);
    return new DrivePerMeterAuto(this.driveBase, 1);
  }
}
