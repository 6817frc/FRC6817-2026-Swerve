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
import com.revrobotics.PersistMode;
import frc.robot.utils.Ports;

public class Climber extends SubsystemBase {
  
  //These are the motors used in this subsystem
  public final SparkMax m_climbLead;
  public final SparkMax m_climbFollow;
  public final SparkClosedLoopController climbPID;
  public final AbsoluteEncoder climbEncoder;

  /** Creates a new Climber. */
  public Climber() {

    //This sets the configuration for the lead motor controlling the climber
    m_climbLead = new SparkMax(Ports.CAN.Climb1Lead, MotorType.kBrushless);
    SparkMaxConfig climbLeadConfig = new SparkMaxConfig();
    climbLeadConfig.inverted(false).idleMode(IdleMode.kBrake);
    climbLeadConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(0.3, 0, 0);
    climbPID = m_climbLead.getClosedLoopController();
    climbEncoder = m_climbLead.getAbsoluteEncoder();

    //This sets the configuration of the motor following the other climb motor
    m_climbFollow = new SparkMax(Ports.CAN.Climb1follow, MotorType.kBrushless);
    SparkMaxConfig climbFollowConfig = new SparkMaxConfig();
    climbFollowConfig.follow(m_climbLead, true).idleMode(IdleMode.kBrake);

    m_climbLead.configure(climbLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_climbFollow.configure(climbFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /* Variables for climb positions and general */
  double upPos = 0.5; //TODO change to real value
  double downPos = 0; //TODO change to real value
  double trigThreshold = 0.15;
  double climbSpeed = 1; // change if it needs to be slower


  /* Functions for moving the climber to positions */

  //moves the climber up to the ladder with position
  public void climbUpPos() {
    climbPID.setSetpoint(upPos, SparkMax.ControlType.kPosition);
  }

  //moves the climber up to the first bar
  public void climbUp(double rightTrig) {
    if (rightTrig > trigThreshold) {
      m_climbLead.set(climbSpeed * rightTrig);
    }
  }

  //moves the climber back down to start position
  public void climbDownPos() {
    climbPID.setSetpoint(downPos, SparkMax.ControlType.kPosition);
  }

  //moves the climber back down
  public void climbDown(double leftTrig) {
    if (leftTrig > trigThreshold) {
      m_climbLead.set(climbSpeed * leftTrig);
    }
  }

  //stops all movement of the climber
  public void stopClimb() {
    m_climbLead.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
