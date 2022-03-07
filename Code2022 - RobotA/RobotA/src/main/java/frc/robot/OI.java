// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Drive;
import frc.robot.commands.ChangeIntakeRotation;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.ClimbBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.IntakeBase;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.StorageSubsystem;

/** Add your docs here. */
public class OI {
    private static Joystick leftJoystick = new Joystick(Constants.leftJoystickPort);
    private static Joystick rightJoystick = new Joystick(Constants.rightJoystickPort);

    private static XboxController xboxController = new XboxController(Constants.xboxControllerPort);

    private JoystickButton aButton = new JoystickButton(xboxController, 1);
    private JoystickButton bButton = new JoystickButton(xboxController, 2);
    private JoystickButton xButton = new JoystickButton(xboxController, 3);
    private JoystickButton yButton = new JoystickButton(xboxController, 4);
    private JoystickButton leftBumber = new JoystickButton(xboxController, 5);
    private JoystickButton rightBumber = new JoystickButton(xboxController, 6);
    
    private static JoystickButton righJoystickButton = new JoystickButton(rightJoystick, 1);
    private static JoystickButton leftJoystickButton = new JoystickButton(leftJoystick, 1);

    private DriveBase driveBase;
    private ShooterBase shooterBase;
    private ClimbBase climbBase;
    private StorageSubsystem storageSubsystem;
    private IntakeBase intakeBase;


    public static XboxController getXboxController(){
        return xboxController;
    }
    public static boolean getBackButton(){
        return xboxController.getBackButton();
    }

    public static boolean getRightJoystickButton(){
        return righJoystickButton.get();
    }
    public static boolean getLeftJoystickButton(){
        return leftJoystickButton.get();
    }

    public static double getLeftJoystick(){
        return (leftJoystick.getY() > Constants.OIDeadzone || leftJoystick.getY() < -Constants.OIDeadzone)?(-leftJoystick.getY()):(0);
    }
    public static double getRightJoystick(){
        return (rightJoystick.getY() > Constants.OIDeadzone || rightJoystick.getY() < -Constants.OIDeadzone)?(-rightJoystick.getY()):(0);
    }

    public OI(StorageSubsystem storageSubsystem, IntakeBase intakeBase, DriveBase driveBase,ShooterBase shooterBase, ClimbBase climbBase){
        this.driveBase = driveBase;
        this.intakeBase = intakeBase;
        this.shooterBase = shooterBase;
        this.storageSubsystem = storageSubsystem;
        this.climbBase = climbBase;
        this.aButton.whenPressed(new ChangeIntakeRotation(this.intakeBase));
        // this.leftBumber.whenPressed(new LockOnHub(RobotContainer.driveBase, RobotContainer.limelightBase));
        
        this.rightBumber.whenPressed(new SetShooterSpeed(this.shooterBase, Constants.shootingFromHubVelocity,this.storageSubsystem));       
    }
}
