// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;

import frc.robot.Constants.FieldConstants;
import frc.robot.utils.Ports;
import frc.robot.utils.Utils;

public class Shooter extends SubsystemBase {

  // These are the motors used in this subsystem
  public final SparkMax m_shooterLaunchLead;
  public final SparkMax m_shooterLaunchFollow;
  public final SparkMax m_shooterTilt;
  public final SparkMax m_shooterIndexer;
  public final SparkClosedLoopController tiltPID;
  public final AbsoluteEncoder tiltEncoder;

  /** Creates a new Shooter. */
  public Shooter() {

    // This sets the configuration for the motor launching the fuel
    m_shooterLaunchLead = new SparkMax(Ports.CAN.shooterLaunchLead, MotorType.kBrushless);
    SparkMaxConfig launchConfigLead = new SparkMaxConfig();
    launchConfigLead.inverted(false).idleMode(IdleMode.kCoast);

    // This sets the configuration for the following motor launching the fuel
    m_shooterLaunchFollow = new SparkMax(Ports.CAN.shooterLaunchFollow, MotorType.kBrushless);
    SparkMaxConfig launchConfigFollow = new SparkMaxConfig();
    launchConfigFollow.follow(m_shooterLaunchLead, true).idleMode(IdleMode.kCoast);

    // This sets the configuration for the motor tilting the head of the shooter
    m_shooterTilt = new SparkMax(Ports.CAN.shooterTilt, MotorType.kBrushless);
    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.inverted(false).idleMode(IdleMode.kBrake);
    tiltConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.5, 0, 0);
    tiltPID = m_shooterTilt.getClosedLoopController();
    tiltEncoder = m_shooterTilt.getAbsoluteEncoder();

    // This sets the configuration for the indexer feeding into the shooter
    m_shooterIndexer = new SparkMax(Ports.CAN.shooterIndex, MotorType.kBrushless);
    SparkMaxConfig indexConfig = new SparkMaxConfig();
    indexConfig.inverted(false).idleMode(IdleMode.kBrake);

    m_shooterLaunchLead.configure(launchConfigLead, ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
    m_shooterLaunchFollow.configure(launchConfigFollow, ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
    m_shooterTilt.configure(tiltConfig, ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
    m_shooterIndexer.configure(indexConfig, ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  /* Variables for the shooter functions */
  double launchSpeed = 0.9; // TODO change to real value
  double launchPos = 0.7; // TODO change to real value
  double indexVel = 0.75; // TODO change to real value

  /* Functions for launching movements */

  /** launch fuel at specified speed (-1.0 to 1.0) */
  public void shoot(double speed) {
    m_shooterLaunchLead.set(speed);
  }

  /** launch fuel based on distance from hub */
  public void shoot(Pose2d botPose) {
    Pose2d hubPose = Utils.redToAllianceSpecific(new Pose2d(FieldConstants.RED_HUB, new Rotation2d()));

    double distance = Math
        .sqrt(Math.pow(hubPose.getX() - botPose.getX(), 2) + Math.pow(hubPose.getY() - botPose.getY(), 2));

    // TODO: get data to set equation
    shoot(0.125 * distance + 0.5);
  }

  /** launch fuel at a predifined speed */
  public void shoot() {
    shoot(launchSpeed);
  }

  // moves launcher backwards
  public void returnFuel() {
    m_shooterLaunchLead.set(-launchSpeed);
  }

  /** stops all launcher movement */
  public void stopLaunch() {
    m_shooterLaunchLead.set(0);
  }

  /* Functions for tilting the launch head */

  // moves to launch position
  public void moveToLaunchPos() {
    tiltPID.setSetpoint(launchPos, ControlType.kPosition);
  }

  /**
   * 
   * @param angle the angle to set the shooter to in degrees (0 is straight
   *              forward)
   */
  public void setLaunchAngle(double angle) {
    double angleDegrees = 0.01 * angle; // TODO: get data to set function
    tiltPID.setSetpoint(angleDegrees, ControlType.kPosition);
  }

  public void moveHood(double speed) {
    tiltPID.setSetpoint(tiltEncoder.getPosition() + speed, ControlType.kPosition);
  }

  // stops the head from tilting
  public void stopTilt() {
    tiltPID.setSetpoint(tiltEncoder.getPosition(), ControlType.kPosition);
  }

  /* Functions for the indexer to move */

  // moves the fuel into the shooter
  public void inIndex() {
    m_shooterIndexer.set(-indexVel);
  }

  // moves the fuel out of the shooter
  public void outIndex() {
    m_shooterIndexer.set(indexVel);
  }

  // stops indexer
  public void stopIndex() {
    m_shooterIndexer.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
