// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.utils.AbsoluteEncoder;
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

    //This sets the configuration of the motor following the other climb motor
    m_climbFollow = new SparkMax(Ports.CAN.Climb1follow, MotorType.kBrushless);
    SparkMaxConfig climbFollowConfig = new SparkMaxConfig();
    climbFollowConfig.follow(m_climbLead, true).idleMode(IdleMode.kBrake);

    m_climbLead.configure(climbLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_climbFollow.configure(climbFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /* Functions for moving the climber to positions */

  //moves the climber up to the first bar
  public void climb1() {
    //TODO add code for the climber to move to the first position
  }

  //moves the climber back down to start position
  public void climbDown() {
    //TODO add code for the climber to move into safe positon for start
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
