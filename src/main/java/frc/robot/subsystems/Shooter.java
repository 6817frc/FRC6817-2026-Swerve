// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import frc.robot.utils.Ports;

public class Shooter extends SubsystemBase {
  
  //These are the motors used in this subsystem
  public final SparkMax m_shooterLaunch;
  public final SparkMax m_shooterTilt;
  public final SparkMax m_shooterIndexer;
  public final SparkClosedLoopController tiltPID;
  public final AbsoluteEncoder tiltEncoder;
  
  /** Creates a new Shooter. */
  public Shooter() {

    //This sets the configuration for the motor launching the fuel
    m_shooterLaunch = new SparkMax(Ports.CAN.shooterLauch, MotorType.kBrushless);
    SparkMaxConfig launchConfig = new SparkMaxConfig();
    launchConfig.inverted(false).idleMode(IdleMode.kBrake);

    //This sets the configuration for the motor tilting the head of the shooter
    m_shooterTilt = new SparkMax(Ports.CAN.shooterTilt, MotorType.kBrushless);
    SparkMaxConfig tiltConfig = new SparkMaxConfig();
    tiltConfig.inverted(false).idleMode(IdleMode.kBrake);
    tiltConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(.3, 0, 0).maxOutput(.15).minOutput(-0.15);
    tiltPID = m_shooterTilt.getClosedLoopController();
    tiltEncoder = m_shooterTilt.getAbsoluteEncoder();

    //This sets the configuration for the indexer feeding into the shooter
    m_shooterIndexer = new SparkMax(Ports.CAN.shooterIndex, MotorType.kBrushless);
    SparkMaxConfig indexConfig = new SparkMaxConfig();
    indexConfig.inverted(false).idleMode(IdleMode.kBrake);

    m_shooterLaunch.configure(launchConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    m_shooterTilt.configure(tiltConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    m_shooterIndexer.configure(indexConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
  }

  /* Functions for launching movements */

  //launched fuel based on joystick input
  public void launch() {
    //TODO add code for the shooter to launch fuel and score
  }

  //moves launcher backwards
  public void returnFuel() {
    //TODO if needed add code for the launcher to move the other way
  }  

  //stops all launcher movement
  public void stopLaunch() {
    m_shooterLaunch.set(0);
  }

  /* Functions for tilting the launch head */

  //moves to launch position
  public void launchPos() {
    //TODO add code for the position the head needs to be in to launch
  }

  //moves head upwards with joysticks
  public void upTilt() {
    //TODO add code for moving the head upwards
  }

  //moves head downwards with joysticks
  public void downTilt() {
    //TODO add code for moving the head downwards
  }

  //stops the head from tilting
  public void stopTilt() {
    m_shooterTilt.set(0);
  }

  /* Functions for the indexer to move */

  //moves the fuel into the shooter
  public void inIndex() {
    //TODO add code for the fuel to move into shooter
  }

  //moves the fuel out of the shooter
  public void outIndex() {
    //TODO add code to move the fuel out of the shooter if needed
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
