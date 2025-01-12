// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

public class RobotContainer {

  //declaring controllers
  //operator controller can be plugged in, but is not currently used
  private final CommandXboxController m_DriveController = new CommandXboxController(0);


  //options for side sendable chooser
  public static final String m_blue = "Blue";
  public static final String m_red = "Red";

  //variable affected by sendable choosers from smart dash
  public String m_autoSelected;
  public String m_sideChosen;

  //declaration of sendable choosers
  //m_auto_chooser's options are prebuilt with path planner
  //do not try to add yourself or it breaks
  public final SendableChooser<String> m_side_chooser = new SendableChooser<>();

  // declaring the axes for the drivers controller that affect teleop swerve command
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  //triggers that affect modes in teleop swerve command
  private final Trigger robotCentric =
  new Trigger(m_DriveController.leftBumper());

  private final Trigger rotation_snap_pressed =
  new Trigger(m_DriveController.rightBumper());  

  private final Trigger strafe_snap_pressed =
  new Trigger(m_DriveController.a());

  //for smart dash, affects the flipping of the path
  public final boolean blueOrNot = true;
  
  //Subsystems 
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final Limelight m_Limelight = new Limelight();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_SwerveSubsystem.LEDoff();

    //named commands for autos, make sure the name aligns with pathplanner named command
    
    //adding the option to the choosers
    m_side_chooser.setDefaultOption("Blue", m_blue);
    m_side_chooser.addOption("Red", m_red);


    //displaying choosers on smart dash
    SmartDashboard.putData("Side", m_side_chooser);
    //swerve command to drive during tele-op
    m_SwerveSubsystem.setDefaultCommand(
      new TeleopSwerve(
          m_SwerveSubsystem, m_Limelight,
          () -> -m_DriveController.getRawAxis(translationAxis),
          () -> -m_DriveController.getRawAxis(strafeAxis),
          () -> -m_DriveController.getRawAxis(rotationAxis),
          () -> m_DriveController.getRawAxis(XboxController.Axis.kLeftTrigger.value),
          () -> robotCentric.getAsBoolean(),
          () -> rotation_snap_pressed.getAsBoolean(),
          () -> strafe_snap_pressed.getAsBoolean(),
          () -> blueOrNot));

    //configure the controller bindings
    configureBindings();
  }

  private void configureBindings() {

    m_DriveController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    
    //m_DriveController.button(Button.kA.value).onTrue(new Toggle(2, m_Shooter));
    //m_DriveController.button(Button.kB.value).onTrue(new Toggle(1, m_Shooter));
  }

  //turns off robot LEDs
  public void turnOffLEDS()
  {
    m_SwerveSubsystem.LEDoff();
  }

  //moves wheels to their straightforward positions
  public void resetWheels()
  {
    m_SwerveSubsystem.setWheelsToX();
  }

  //returns the side chosen for auto in smart dash
  public String getSide() {
    m_sideChosen = m_side_chooser.getSelected();
    return m_sideChosen;
  }

  //returns the command chosen for auto in smart dash
  public Command getAutonomousCommand() {
    return null;
  }
}

