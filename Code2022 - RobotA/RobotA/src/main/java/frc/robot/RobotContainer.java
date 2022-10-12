// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimToHub;
import frc.robot.commands.AutoDriveToHub;
import frc.robot.commands.Autonomous3Balls;
import frc.robot.commands.Autonomous3BallsAndPickup;
import frc.robot.commands.Autonomous3BallsNew;
import frc.robot.commands.AutonomousOneBallPickup;
import frc.robot.commands.AutonomousTaxiPickUp;
import frc.robot.commands.CollectBall;
import frc.robot.commands.DriveForward;
import frc.robot.commands.TeleopShoot;
import frc.robot.commands.Turn180Degrees;
import frc.robot.subsystems.ClimbBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.LEDSubsystem;
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
  public final LimelightBase limelightBase = new LimelightBase();
  public final LEDSubsystem ledSubsystem = new LEDSubsystem();
  public final OI robot_oi = new OI(storageSubsystem, intakeBase, driveBase, shooterBase, climbBase, limelightBase);
  
  Trajectory t = new Trajectory();

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
    //return new InstantCommand(()-> driveBase.setVelocity(1,1), driveBase).andThen(new WaitCommand(1.5), new InstantCommand(()-> driveBase.print(),driveBase),
    //new InstantCommand(()-> driveBase.setPower(0,0), driveBase));
    //return new AutonomousOneBallPickup(this.driveBase, this.shooterBase, this.limelightBase, this.intakeBase, this.storageSubsystem);
    //return new AutoDriveToHub(this.driveBase, this.limelightBase);
    //return new Turn180Degrees(driveBase);

    //return new InstantCommand(()->this.intakeBase.intakeIn(), this.intakeBase);

    //return new Autonomous3Balls(driveBase, intakeBase, storageSubsystem, shooterBase, limelightBase);

    // return new ChangeMapPose(driveBase);
    
    
    // RamseteCommand command = new RamseteCommand(
    //   t,
    //   driveBase::getPose,
    //   new RamseteController(2, 0.7),
    //   driveBase.getKinematics(),
    //   driveBase::setVelocity,
    //   driveBase
    //   );

    //return new AimToHub(limelightBase, driveBase);

    // return new TeleopShoot(driveBase, shooterBase, limelightBase, storageSubsystem);



    // return new AutonomousTaxiPickUp(driveBase, shooterBase, limelightBase, intakeBase, storageSubsystem);
    // return new AutonomousOneBallPickup(this.driveBase, this.shooterBase, this.limelightBase, this.intakeBase, this.storageSubsystem);
    // return new Autonomous3Balls(driveBase, intakeBase, storageSubsystem, shooterBase, limelightBase);
    return new Autonomous3BallsNew(driveBase, intakeBase, storageSubsystem, shooterBase, limelightBase);
    // return new AutonomousTaxiPickUp(driveBase, shooterBase, limelightBase, intakeBase, storageSubsystem);
   // return new Autonomous3BallsAndPickup(driveBase, intakeBase, storageSubsystem, shooterBase, limelightBase);
      
    //   return new InstantCommand(()->driveBase.SetPose(t.getInitialPose()), driveBase).andThen(command);
  }
}
