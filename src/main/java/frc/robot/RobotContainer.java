// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.Ports;
import frc.robot.utils.Utils;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OutreachTargetingConstants;

import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Debug;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  public static final double JOYSTICK_AXIS_THRESHOLD = 0.05;

  public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();
  public final Climber climb = new Climber();
  public final Debug debug = new Debug();

  public final Field2d field = new Field2d();

  public final SendableChooser<Integer> autoChooser = new SendableChooser<>();

  private double speedMult = 0.75;
  private double triggerThreshold = 0.05;

  private boolean useAutoDrive = false;
  private boolean useAutoTurn = false;

  // Allow the copilot controller to move the robot, for use in outreach mode
  private boolean useCopilot = false;

  private boolean useOutreachTargetEditing = false;
  private Pose3d outreachTarget = new Pose3d();
  private double maxHeight = 1.5;

  // Variables for the outreach target shooting calculations
  /**
   * The horizontal distance from the robot to the outreachTarget
   */
  private double outreachTargetDistance = 0.0;
  /**
   * The unclamped height at the target x without gravity
   */
  private double idealStraightLineHeight = 0.0;
  /**
   * The square of the unclamped height at the target x without gravity
   */
  private double idealStraightLineHeightSq = 0.0;
  /**
   * The unclamped inverse of the time to reach the straightLineHeight
   *
   * This number is then multiplied by a square distance to get a square velocity
   */
  private double idealInverseTimeSquared = 0.0;
  /**
   * The clamped height at the target x without gravity
   */
  private double straightLineHeight = 0.0;
  /**
   * The square of the clamped height at the target x without gravity
   */
  private double straightLineHeightSq = 0.0;
  /**
   * The clamped inverse of the time to reach the straightLineHeight
   *
   * This number is then multiplied by a square distance to get a square velocity
   */
  private double inverseTimeSquared = 0.0;
  /**
   * The slope of the unclamped launch angle
   */
  private double idealLaunchSlope = 0.0;
  private double launchAngle = 0.0;
  private double launchVelocity = 0.0;

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
    configureDefaultCommands();
    // Configure the controller bindings
    configureMainBindings();
    // Add auto chooser to smartdashboard
    configureAutoChooser();
    // Configure commands for use in path planner
    configureAutoCommands();
    SmartDashboard.putNumber("Launcher Speed", 0.6);
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new RunCommand(() -> {
      getDriveValues();
      drivetrain.drive(-leftStickX, leftStickY, -rightStickX, true, false, useAutoDrive, useAutoTurn);
    }, drivetrain));
    debug.setDefaultCommand(new RunCommand(() -> {
      switch (debug.checkControllerBindingsUpdate()) {
        case NONE:
          break;
        case MAIN:
          configureMainBindings();
          break;
        case OUTREACH:
          configureOutreachBindings();
          break;
      }
    }, debug));
  }

  public void configureMainBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    /* --------------------------- Driver Controller --------------------------- */

    // driverController.x() // button:X - Set the pose based on tag
    // .onTrue(Commands.runOnce(() -> {
    // drivetrain.setPoseFromTag();
    // useAutoDrive = true;
    // useAutoTurn = true;
    // }))
    // .onFalse(Commands.runOnce(() -> {
    // useAutoDrive = false;
    // useAutoTurn = false;
    // }));

    driverController.y().onTrue(Commands.runOnce(() -> drivetrain.zeroHeading())); // button:Y - Reset field orientation

    // Point toward the center of the hub
    driverController.a()
        .whileTrue(Commands.run(() -> {
          drivetrain.setIdealRotation(Utils.directionToPose(drivetrain.getPose(),
              Utils.redToAllianceSpecific(new Pose2d(FieldConstants.RED_HUB, new Rotation2d()))));
          useAutoTurn = true;
        }))
        .onFalse(Commands.runOnce(() -> useAutoTurn = false));

    driverController.b()
        .onTrue(Commands.runOnce(() -> intake.intakeFuel()))
        .onFalse(Commands.runOnce(() -> intake.stopWheels()));

    driverController.x()
        .onTrue(Commands.runOnce(() -> intake.outtakeFuel()))
        .onFalse(Commands.runOnce(() -> intake.stopWheels()));

    // Go to specified position
    driverController.povLeft()
        .onTrue(Commands.runOnce(() -> {
          // 3 meters and 25 degrees from the red hub on the left side
          drivetrain.setIdealPose(new Pose2d(14.63, 2.76, Rotation2d.fromDegrees(155)), true);
          useAutoDrive = true;
          useAutoTurn = true;
        }))
        .onFalse(Commands.runOnce(() -> {
          useAutoDrive = false;
          useAutoTurn = false;
        }));

    driverController.povDown()
        .onTrue(Commands.runOnce(() -> {
          // 3 meters striaght back from the hub
          drivetrain.setIdealPose(new Pose2d(14.91, 4.03, Rotation2d.fromDegrees(180)), true);
          useAutoDrive = true;
          useAutoTurn = true;
        }))
        .onFalse(Commands.runOnce(() -> {
          useAutoDrive = false;
          useAutoTurn = false;
        }));

    driverController.povRight()
        .onTrue(Commands.runOnce(() -> {
          // 3 meters and 25 degrees from the red hub on the right side
          drivetrain.setIdealPose(new Pose2d(14.63, 5.3, Rotation2d.fromDegrees(205)), true);
          useAutoDrive = true;
          useAutoTurn = true;
        }))
        .onFalse(Commands.runOnce(() -> {
          useAutoDrive = false;
          useAutoTurn = false;
        }));

    driverController.rightTrigger(triggerThreshold).onTrue(Commands.runOnce(() -> intake.armDown()));
    driverController.rightTrigger(triggerThreshold).onFalse(Commands.runOnce(() -> intake.armMid()));

    driverController.leftTrigger(triggerThreshold).onTrue(Commands.runOnce(() -> intake.armUp()));

    driverController.rightBumper().onTrue(Commands.runOnce(() -> intake.armFullUp()));

    /* --------------------------- Copilot Controller --------------------------- */

    copilotController.povUp().onTrue(Commands.runOnce(() -> shooter.setLaunchAngle(55)));
    copilotController.povLeft().onTrue(Commands.runOnce(() -> shooter.setLaunchAngle(58)));
    copilotController.povRight().onTrue(Commands.runOnce(() -> shooter.setLaunchAngle(62)));
    copilotController.povDown().onTrue(Commands.runOnce(() -> shooter.setLaunchAngle(65)));

    copilotController.x()
        .onTrue(Commands.runOnce(() -> shooter.outIndex()))
        .onFalse(Commands.runOnce(() -> shooter.stopIndex()));
    copilotController.y()
        .onTrue(Commands.runOnce(() -> shooter.inIndex()))
        .onFalse(Commands.runOnce(() -> shooter.stopIndex()));

    copilotController.a() // Shoot at a set speed
        .whileTrue(Commands.run(() -> shooter.shoot()))
        .onFalse(Commands.runOnce(() -> shooter.idleShoot()));

    copilotController.b() // Change hood angle and launch height based on distance from hub
        .whileTrue(Commands.run(() -> shooter.shoot(drivetrain.getPose())))
        .onFalse(Commands.runOnce(() -> shooter.idleShoot()));

    copilotController.rightBumper()
        .onTrue(Commands.runOnce(() -> intake.intakeFuel()))
        .onFalse(Commands.runOnce(() -> intake.stopWheels()));

    copilotController.leftBumper()
        .onTrue(Commands.runOnce(() -> intake.outtakeFuel()))
        .onFalse(Commands.runOnce(() -> intake.stopWheels()));

  }

  public void configureOutreachBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    /* --------------------------- Driver Controller --------------------------- */

    driverController.y().onTrue(Commands.runOnce(() -> drivetrain.zeroHeading())); // button:Y - Reset field orientation

    driverController.a() // Shoot at a set speed
        .whileTrue(Commands.run(() -> shooter.shoot(SmartDashboard.getNumber("Launcher Speed", 0.6))))
        .onFalse(Commands.runOnce(() -> shooter.stopLaunch()));

    // Point at the hub and auto shoot
    driverController.b()
        .onTrue(Commands.runOnce(() -> useAutoTurn = true))
        .whileTrue(Commands.run(() -> {
          drivetrain.setIdealRotation(Utils.directionToPose(drivetrain.getPose(), outreachTarget.toPose2d()));
          setShootValues(drivetrain.getPose(), outreachTarget.toPose2d());
          shooter.outreachShoot(launchAngle, launchVelocity);
        }))
        .onFalse(Commands.runOnce(() -> {
          useAutoTurn = false;
          shooter.stopLaunch();
        }));

    driverController.x().onTrue(Commands.runOnce(() -> {
      useCopilot = !useCopilot;
    }));

    driverController.start().onTrue(Commands.runOnce(() -> {
      useOutreachTargetEditing = !useOutreachTargetEditing;
    }));

    driverController.back().onTrue(Commands.runOnce(() -> setOutreachTarget(new Pose3d(drivetrain.getPose()), 0)));

    driverController.povUp()
        .onTrue(Commands.runOnce(() -> shooter.inIndex()))
        .onFalse(Commands.runOnce(() -> shooter.stopIndex()));

    driverController.povDown()
        .onTrue(Commands.runOnce(() -> shooter.outIndex()))
        .onFalse(Commands.runOnce(() -> shooter.stopIndex()));

    // Right trigger pressed: down and intake in
    // Right trigger release: mid pos and stop intake
    driverController.rightTrigger(triggerThreshold).onTrue(Commands.runOnce(() -> {
      if (!useOutreachTargetEditing)
        intake.armDown();
    }));
    driverController.rightTrigger(triggerThreshold).onFalse(Commands.runOnce(() -> {
      if (!useOutreachTargetEditing)
        intake.armMid();
    }));

    // Left trigger: arm up
    driverController.leftTrigger(triggerThreshold).onTrue(Commands.runOnce(() -> {
      if (!useOutreachTargetEditing)
        intake.armUp();
    }));

    // Right bumper: arm all the way up
    driverController.rightBumper().onTrue(Commands.runOnce(() -> intake.armFullUp()));

    /* --------------------------- Copilot Controller --------------------------- */

  }

  /**
   * This section is used to calculate the speed multiplier and apply that as well
   * as a deadband to the controller's joysticks
   */
  private void getDriveValues() {
    leftStickX = MathUtil.applyDeadband(driverController.getLeftX(), JOYSTICK_AXIS_THRESHOLD);
    leftStickY = MathUtil.applyDeadband(driverController.getLeftY(), JOYSTICK_AXIS_THRESHOLD);
    rightStickX = MathUtil.applyDeadband(driverController.getRightX(), JOYSTICK_AXIS_THRESHOLD);

    if (useOutreachTargetEditing) {
      double rightStickY = MathUtil.applyDeadband(driverController.getRightY(), JOYSTICK_AXIS_THRESHOLD);
      double leftTrigger = MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), triggerThreshold);
      double rightTrigger = MathUtil.applyDeadband(driverController.getRightTriggerAxis(), triggerThreshold);
      setOutreachTarget(leftStickX, leftStickY, rightStickY, rightTrigger - leftTrigger);

      leftStickX = 0;
      leftStickY = 0;
      rightStickX = 0;
      return;
    }

    speedMult = 1;
    if (useCopilot) {
      if (Math.abs(leftStickX) > 0 || Math.abs(leftStickY) > 0 || Math.abs(rightStickX) > 0) {
        // Automatically take control when the driver moves the sticks
        speedMult *= driverController.leftBumper().getAsBoolean() ? 0.25 : 0.75;
      } else {
        // Allow the copilot controller to move the robot
        leftStickX = MathUtil.applyDeadband(copilotController.getLeftX(), JOYSTICK_AXIS_THRESHOLD);
        leftStickY = MathUtil.applyDeadband(copilotController.getLeftY(), JOYSTICK_AXIS_THRESHOLD);
        rightStickX = MathUtil.applyDeadband(copilotController.getRightX(), JOYSTICK_AXIS_THRESHOLD);

        speedMult *= 0.2;
      }
    } else {
      speedMult *= driverController.leftBumper().getAsBoolean() ? 0.25 : 1;
    }

    SmartDashboard.putNumber("Speed Mult", speedMult);

    // Apply the speed multiplier
    leftStickX = leftStickX * speedMult;
    leftStickY = leftStickY * speedMult;
    rightStickX = rightStickX * speedMult;
  }

  public void setOutreachTarget(double leftStickX, double leftStickY, double rightStickY, double triggerDiff) {
    // ChassisSpeeds.fromRobotRelativeSpeeds(x, y, 0, 0)
    setOutreachTarget(outreachTarget.plus(new Transform3d(
        -leftStickY * OutreachTargetingConstants.JOYSTICK_MULTIPLIER,
        leftStickX * OutreachTargetingConstants.JOYSTICK_MULTIPLIER,
        -rightStickY * OutreachTargetingConstants.JOYSTICK_MULTIPLIER,
        new Rotation3d())),
        triggerDiff * OutreachTargetingConstants.TRIGGER_MULTIPLIER);
  }

  public void setOutreachTarget(Pose3d targetPose, double heightDelta) {
    outreachTarget = targetPose;
    maxHeight += heightDelta;

    idealStraightLineHeight = 2 * (maxHeight + Math.sqrt(maxHeight * (maxHeight - outreachTarget.getZ())));
    idealStraightLineHeightSq = idealStraightLineHeight * idealStraightLineHeight;
    idealInverseTimeSquared = OutreachTargetingConstants.halfGravity
        / (idealStraightLineHeight - outreachTarget.getZ());

    SmartDashboard.putString("Outreach Target", outreachTarget.toString());
    SmartDashboard.putNumber("Max Height", maxHeight);
  }

  public void setShootValues(Pose2d botPose, Pose2d targetPose) {
    outreachTargetDistance = botPose.getTranslation().getDistance(targetPose.getTranslation());
    idealLaunchSlope = idealStraightLineHeight / outreachTargetDistance;
    if (idealLaunchSlope <= OutreachTargetingConstants.minSlope) {
      // Recalculate inverseTimeSquared and straightLineHeight
      straightLineHeight = outreachTargetDistance * OutreachTargetingConstants.minSlope;
      straightLineHeightSq = straightLineHeight * straightLineHeight;
      inverseTimeSquared = OutreachTargetingConstants.halfGravity / (straightLineHeight - outreachTarget.getZ());

      launchAngle = OutreachTargetingConstants.minAngle;
      launchVelocity = Math
          .sqrt(inverseTimeSquared * (outreachTargetDistance * outreachTargetDistance + straightLineHeightSq));
      SmartDashboard.putNumberArray("min values", new double[] { outreachTargetDistance, idealLaunchSlope,
          straightLineHeight, straightLineHeightSq, inverseTimeSquared });
    } else if (idealLaunchSlope < OutreachTargetingConstants.maxSlope) {
      launchAngle = Math.toDegrees(Math.atan(idealLaunchSlope));
      // straightLineHeight is equivalent to the idealStraightLineHeight, so all the
      // ideal values work
      launchVelocity = Math.sqrt(
          idealInverseTimeSquared * (outreachTargetDistance * outreachTargetDistance + idealStraightLineHeightSq));
      SmartDashboard.putNumberArray("mid values", new double[] { outreachTargetDistance, idealLaunchSlope,
          idealStraightLineHeight, idealStraightLineHeightSq, idealInverseTimeSquared });
    } else {
      // Recalculate inverseTimeSquared and straightLineHeight
      straightLineHeight = outreachTargetDistance * OutreachTargetingConstants.maxSlope;
      straightLineHeightSq = straightLineHeight * straightLineHeight;
      inverseTimeSquared = OutreachTargetingConstants.halfGravity / (straightLineHeight - outreachTarget.getZ());

      launchAngle = OutreachTargetingConstants.maxAngle;
      launchVelocity = Math
          .sqrt(inverseTimeSquared * (outreachTargetDistance * outreachTargetDistance + straightLineHeightSq));
      SmartDashboard.putNumberArray("max values", new double[] { outreachTargetDistance, idealLaunchSlope,
          straightLineHeight, straightLineHeightSq, inverseTimeSquared });
    }
    SmartDashboard.putNumber("Outreach Launch Angle", launchAngle);
    SmartDashboard.putNumber("Outreach Launch Velocity", launchVelocity);
  }

  public void configureAutoCommands() {
    NamedCommands.registerCommand("moveHoodToLaunchPos", Commands.runOnce(() -> shooter.moveToLaunchPos()));
    NamedCommands.registerCommand("shoot", Commands.runOnce(() -> shooter.shoot()));
    NamedCommands.registerCommand("stopShoot", Commands.runOnce(() -> shooter.stopLaunch()));
    NamedCommands.registerCommand("autoShoot", Commands.runOnce(() -> shooter.shoot(drivetrain.getPose())));
    NamedCommands.registerCommand("inIndex", Commands.runOnce(() -> shooter.inIndex()));
    NamedCommands.registerCommand("outIndex", Commands.runOnce(() -> shooter.outIndex()));
    NamedCommands.registerCommand("stopIndex", Commands.runOnce(() -> shooter.stopIndex()));
    NamedCommands.registerCommand("stop", Commands.runOnce(() -> drivetrain.stop()));
    NamedCommands.registerCommand("inIntake", Commands.runOnce(() -> intake.intakeFuel()));
    NamedCommands.registerCommand("outIntake", Commands.runOnce(() -> intake.outtakeFuel()));
    NamedCommands.registerCommand("stopIntake", Commands.runOnce(() -> intake.stopWheels()));
    NamedCommands.registerCommand("armUp", Commands.runOnce(() -> intake.armUp()));
    NamedCommands.registerCommand("armMid", Commands.runOnce(() -> intake.armMid()));
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Nothing", -1);
    for (int i = 0; i < AutoConstants.AutoPaths.length; ++i) {
      autoChooser.addOption(AutoConstants.AutoPaths[i], i);
    }

    SmartDashboard.putData("Auto Choices", autoChooser);
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
