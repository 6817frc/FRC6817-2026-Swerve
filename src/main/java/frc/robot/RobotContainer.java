// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.Ports;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

  public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  public final Field2d field = new Field2d();

  public final SendableChooser<Boolean> usePoseEstimateChooser = new SendableChooser<>();

  private double speedMult = 1;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
  private final CommandXboxController copilotController = 
      new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);

  private double leftStickX = 0;
  private double leftStickY = 0;
  private double rightStickX = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(new RunCommand(() -> {
      getDriveValues();
      drivetrain.drive(-leftStickX, leftStickY, -rightStickX);
    }, drivetrain));

    driverController.x().onTrue(Commands.runOnce(() -> drivetrain.setL2Pose())); // button:X - Set the pose based on the tag
    driverController.x().whileTrue(Commands.run(() -> drivetrain.goToIdealPose())); // Go to the pose
    driverController.x().onFalse(Commands.runOnce(() -> drivetrain.resetOffsets())); // Reset
    
    driverController.y().onTrue(Commands.runOnce(() -> drivetrain.zeroHeading())); // button:Y - Reset field orientation

    driverController.start().whileTrue(Commands.run(() -> drivetrain.faceTowardTag())); // buttons:Start - face the robot toward the tag
    driverController.start().onFalse(Commands.runOnce(() -> drivetrain.resetOffsets())); // Reset turn offset
  }

  private void getDriveValues() {
    speedMult = 1 - driverController.getRightTriggerAxis() * 0.75 - driverController.getLeftTriggerAxis() * 0.2;

    leftStickX = MathUtil.applyDeadband(driverController.getLeftX(), JOYSTICK_AXIS_THRESHOLD) * speedMult;
    leftStickY = MathUtil.applyDeadband(driverController.getLeftY(), JOYSTICK_AXIS_THRESHOLD) * speedMult;
    rightStickX = MathUtil.applyDeadband(driverController.getRightX(), JOYSTICK_AXIS_THRESHOLD) * speedMult;
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
    // An example command will be run in autonomous
    return null;
  }
}
