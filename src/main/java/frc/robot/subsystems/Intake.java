package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
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

public class Intake extends SubsystemBase{
/** Creates a new Intake. */

  //These are the motors used in the subsystem.
  public final SparkMax m_intakeWheels;
  public final SparkMax m_intakeArm;
  SparkClosedLoopController intakeArmPID;
  AbsoluteEncoder armEncoder;


  public Intake() {

    //This sets the configuration for the motor controlling the wheels of the intake subsystem
    m_intakeWheels = new SparkMax(Ports.CAN.intakeWheels, MotorType.kBrushless);
    SparkMaxConfig wheelConfig = new SparkMaxConfig();
    wheelConfig.inverted(false).idleMode(IdleMode.kBrake);

    //This sets the config for the motor controlling the additional arm movement
    m_intakeArm = new SparkMax(Ports.CAN.intakeArm, MotorType.kBrushless);
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.inverted(true).idleMode(IdleMode.kBrake);
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(0.3, 0.0, 0.0).maxOutput(0.15).minOutput(-0.15);
    intakeArmPID = m_intakeArm.getClosedLoopController();
    armEncoder = m_intakeArm.getAbsoluteEncoder();


    m_intakeWheels.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeArm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
    

    /* Functions for wheel movements: */

    //moves wheels using joystick input
    public void intakeFuel() {
        //TODO add code to move fuel into the robot
    }

    public void outtakeFuel() {
        //TODO add code to move fuel out of the robot if needed
    }

    public void stopWheels() {
        m_intakeWheels.set(0);
    }

    /* Functions for various arm movements: */

    //moves intake arm to the down position
    public void armDown() {
        //TODO add code to set position for arm using .setSetpoint()
    }

    //moves arm to the up position for intake
    public void armUp() {
        //TODO add code to set position for arm
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
