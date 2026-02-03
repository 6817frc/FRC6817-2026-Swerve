// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.Ports;
import frc.robot.Constants.AutoConstants;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

  public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();
  public final Climber climb = new Climber();

  public final Field2d field = new Field2d();

  public final SendableChooser<Integer> autoChooser = new SendableChooser<>();

  private double speedMult = 0.75;
  private double triggerThreshold = 0.15;

  private boolean useAutoDrive = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
  private final CommandXboxController copilotController = new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);

  private double leftStickX = 0;
  private double leftStickY = 0;
  private double rightStickX = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the controller bindings
    configureBindings();
    // Add auto chooser to smartdashboard
    configureAutoChooser();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new RunCommand(() -> {
      getDriveValues();
      if (useAutoDrive) {
        // drivetrain.goToIdealPose();
        drivetrain.drive(-leftStickX, leftStickY, -rightStickX, true, false, true);
      } else {
        drivetrain.drive(-leftStickX, leftStickY, -rightStickX);
      }
    }, drivetrain));

    driverController.x() // button:X - Set the pose based on tag
        .onTrue(Commands.runOnce(() -> {
          drivetrain.setL2Pose();
          useAutoDrive = true;
        }))
        .onFalse(Commands.runOnce(() -> {
          drivetrain.resetOffsets();
          useAutoDrive = false;
        }));

    driverController.y().onTrue(Commands.runOnce(() -> drivetrain.zeroHeading())); // button:Y - Reset field orientation

    driverController.povDown() // dPad:Down - go to specified position
        .onTrue(Commands.runOnce(() -> {
          drivetrain.setIdealPose(new Pose2d(13.449, 2.009, Rotation2d.fromDegrees(-61)), true);
          useAutoDrive = true;
        }))
        .onFalse(Commands.runOnce(() -> {
          drivetrain.resetOffsets();
          useAutoDrive = false;
        }));

    driverController.start().whileTrue(Commands.run(() -> drivetrain.faceTowardTag())) // buttons:Start - face the
                                                                                       // robot toward the tag
        .onFalse(Commands.runOnce(() -> drivetrain.resetOffsets()) // Reset turn offset
        );
    driverController.start().whileTrue(Commands.run(() -> drivetrain.faceTowardTag())); // buttons:Start - face the robot toward the tag
    driverController.start().onFalse(Commands.runOnce(() -> drivetrain.resetOffsets())); // Reset turn offset

    driverController.rightTrigger(triggerThreshold).onTrue(Commands.runOnce(() -> intake.armDown()));
    driverController.rightTrigger(triggerThreshold).onFalse(Commands.runOnce(() -> intake.armUp()));


  
    //copilotController.povUp().onTrue(Commands.runOnce(() -> ));
    copilotController.povLeft().onTrue(Commands.runOnce(() -> shooter.moveToLaunchPos()));
    //copilotController.povRight().onTrue(Commands.runOnce(() -> ));
    //copilotController.povDown().onTrue(Commands.runOnce(() -> )); 
    
    
    copilotController.x().onTrue(Commands.runOnce(() -> shooter.outIndex()));
    copilotController.y().onTrue(Commands.runOnce(() -> shooter.inIndex()));

    copilotController.a().onTrue(Commands.runOnce(() -> shooter.shoot()));

    copilotController.leftBumper().onTrue(Commands.runOnce(() -> climb.climbUpPos()));
    copilotController.rightBumper().onTrue(Commands.runOnce(() -> climb.climbDownPos()));

    copilotController.leftTrigger(triggerThreshold).onTrue(Commands.runOnce(() -> climb.climbDown(copilotController.getLeftTriggerAxis()))); //TODO might not continuously update trigger, so fix that
    copilotController.rightTrigger(triggerThreshold).onTrue(Commands.runOnce(() -> climb.climbUp(copilotController.getRightTriggerAxis()))); //TODO might not continuously update trigger, so fix that
  }

  /* 
   * This section is used to calculate the speed multiplier and apply that as well as a deadband to the controller's joysticks
  */
  private void getDriveValues() {
    if (driverController.rightBumper().getAsBoolean()) {
      speedMult = 1;
    } else if (driverController.leftBumper().getAsBoolean()) {
      speedMult = 0.25;
    } else {
      speedMult = 0.75;
    }

    SmartDashboard.putNumber("Speed Mult", speedMult);

    leftStickX = MathUtil.applyDeadband(driverController.getLeftX(), JOYSTICK_AXIS_THRESHOLD) * speedMult; 
    leftStickY = MathUtil.applyDeadband(driverController.getLeftY(), JOYSTICK_AXIS_THRESHOLD) * speedMult;
    rightStickX = MathUtil.applyDeadband(driverController.getRightX(), JOYSTICK_AXIS_THRESHOLD) * speedMult;
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Nothing", -1);
    for (int i = 0; i < AutoConstants.AutoPaths.length; ++i) {
      autoChooser.addOption(AutoConstants.AutoPaths[i], i);
    }

    SmartDashboard.putData("Auto Choices", autoChooser);
  }

  public SwerveDrivetrain getDriveTrain() {
    return drivetrain;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoChooser.getSelected() == -1) {
      return Commands.none();
    }
    return new PathPlannerAuto(AutoConstants.AutoPaths[autoChooser.getSelected()]);
  }
}
