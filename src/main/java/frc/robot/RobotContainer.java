// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.Ports;
import frc.robot.Constants.AutoConstants;

import com.pathplanner.lib.commands.PathPlannerAuto;

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

  public final Field2d field = new Field2d();

  public final SendableChooser<Integer> autoChooser = new SendableChooser<>();

  private double speedMult = 0.75;

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
      drivetrain.drive(-leftStickX, leftStickY, -rightStickX);
    }, drivetrain));

    driverController.x().onTrue(Commands.runOnce(() -> drivetrain.setL2Pose())) // button:X - Set the pose based on tag
        .whileTrue(Commands.run(() -> drivetrain.goToIdealPose())) // Go to the pose
        .onFalse(Commands.runOnce(() -> drivetrain.resetOffsets())); // Reset

    driverController.y().onTrue(Commands.runOnce(() -> drivetrain.zeroHeading())); // button:Y - Reset field orientation

    driverController.povDown().onTrue(Commands.runOnce(
        () -> drivetrain.idealPose = new Pose2d(13.292746077009944, 2.0681512294440507, Rotation2d.fromDegrees(-61))))
        // dPad:Down - go to position:
        // [13.292746077009944, 2.0681512294440507, -60.90582686418953]
        .whileTrue(Commands.run(() -> drivetrain.goToIdealPose())) // Go to specified pose
        .onFalse(Commands.runOnce(() -> drivetrain.resetOffsets()));

    driverController.start().whileTrue(Commands.run(() -> drivetrain.faceTowardTag())) // buttons:Start - face the
                                                                                       // robot toward the tag
        .onFalse(Commands.runOnce(() -> drivetrain.resetOffsets())); // Reset turn offset
  }

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
