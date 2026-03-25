package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Ports;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  // These are the motors used in the subsystem.
  public final SparkMax m_intakeWheels;
  public final SparkMax m_intakeArm;
  SparkClosedLoopController intakeArmPID;
  AbsoluteEncoder armEncoder;

  public Intake() {

    // This sets the configuration for the motor controlling the wheels of the
    // intake subsystem
    m_intakeWheels = new SparkMax(Ports.CAN.intakeWheels, MotorType.kBrushless);
    SparkMaxConfig wheelConfig = new SparkMaxConfig();
    wheelConfig.inverted(false).idleMode(IdleMode.kBrake);

    // This sets the config for the motor controlling the additional arm movement
    m_intakeArm = new SparkMax(Ports.CAN.intakeArm, MotorType.kBrushless);
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.inverted(false).idleMode(IdleMode.kBrake);
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(1, 0.0, 0.0).maxOutput(0.15).minOutput(-0.15);
    intakeArmPID = m_intakeArm.getClosedLoopController();
    armEncoder = m_intakeArm.getAbsoluteEncoder();

    m_intakeWheels.configure(wheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeArm.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  // variables used in the functions
  double intakeSpeed = 0.25;

  // wheel variables
  double wheelVel = 0.75;

  // Arm Positions
  double downPos = 0.34;
  double midPos = 0.52;
  double upPos = 0.6;

  /* Functions for wheel movements: */

  // stops any movement of the intake wheels
  public void stopWheels() {
    m_intakeWheels.set(0);
  }

  // moves wheels using trigger input
  public void intakeFuel() {
    m_intakeWheels.set(wheelVel);
  }

  // moves the intake wheels the oposite direction in case of stuck fuel
  public void outtakeFuel() {
    m_intakeWheels.set(-wheelVel);
  }

  /* Functions for various arm movements: */

  // moves arm to the up position for intake
  public void armUp() {
    intakeArmPID.setSetpoint(upPos, ControlType.kPosition);
    stopWheels();
  }

  public void armMid() {
    intakeArmPID.setSetpoint(midPos, ControlType.kPosition);
    stopWheels();
  }

  // moves intake arm to the down position
  public void armDown() {
    intakeArmPID.setSetpoint(downPos, ControlType.kPosition);
    intakeFuel();
  }

  public void setArmSpeed(double speed) {
    if (speed == 0)
      stopArm();
    else
      intakeArmPID.setSetpoint(speed, ControlType.kVelocity);
  }

  public void stopArm() {
    intakeArmPID.setSetpoint(armEncoder.getPosition(), ControlType.kPosition);
  }

  public double getArmPosition() {
    return armEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
