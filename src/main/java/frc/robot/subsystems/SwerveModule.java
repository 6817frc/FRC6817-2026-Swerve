package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.AbsoluteEncoder;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final AbsoluteEncoder turnAbsoluteEncoder;
    
    private final SparkClosedLoopController driveClosedLoopController;
    private final SparkClosedLoopController turnClosedLoopController;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingCANId, int turningCANId, int turningAnalogPort) {
		driveConfig = new SparkMaxConfig();
		turnConfig = new SparkMaxConfig();

		driveMotor = new SparkMax(drivingCANId, MotorType.kBrushless);
		turnMotor = new SparkMax(turningCANId, MotorType.kBrushless);

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		driveEncoder = driveMotor.getEncoder();
		turnEncoder = turnMotor.getEncoder();
		turnAbsoluteEncoder = new AbsoluteEncoder(turningAnalogPort);

		driveClosedLoopController = driveMotor.getClosedLoopController();
		turnClosedLoopController = turnMotor.getClosedLoopController();

		driveConfig
			.inverted(false)
			.idleMode(SwerveModuleConstants.DRIVING_MOTOR_IDLE_MODE)
			.smartCurrentLimit(SwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT_AMPS);

		/* Apply position and velocity conversion factors for the driving encoder. 
		The native units for position and velocity are rotations and RPM, respectively,
		but we want meters and meters per second to use with WPILib's swerve APIs. */
		driveConfig.encoder
			.positionConversionFactor(SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION)
			.velocityConversionFactor(SwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM);

		driveConfig.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			// Set the PID gains for the driving motor.
			.pid(SwerveModuleConstants.DRIVING_P, SwerveModuleConstants.DRIVING_I, SwerveModuleConstants.DRIVING_D)
			.velocityFF(SwerveModuleConstants.DRIVING_FF)
			.outputRange(SwerveModuleConstants.DRIVING_MIN_OUTPUT_NORMALIZED, SwerveModuleConstants.DRIVING_MAX_OUTPUT_NORMALIZED);

		driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		
		turnConfig
			.inverted(true)
			.idleMode(SwerveModuleConstants.TURNING_MOTOR_IDLE_MODE)
			.smartCurrentLimit(SwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);

		turnConfig.encoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .positionConversionFactor(SwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION) // radians
            .velocityConversionFactor(SwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM); // radians per second
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(SwerveModuleConstants.TURNING_P, SwerveModuleConstants.TURNING_I, SwerveModuleConstants.TURNING_D)
            .outputRange(SwerveModuleConstants.TURNING_MIN_OUTPUT_NORMALIZED, SwerveModuleConstants.TURNING_MAX_OUTPUT_NORMALIZED)
			.velocityFF(SwerveModuleConstants.TURNING_FF)
			// Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(SwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS, SwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS);
		turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		desiredState.angle = new Rotation2d(turnEncoder.getPosition());
		driveEncoder.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(driveEncoder.getVelocity(),
			new Rotation2d(turnEncoder.getPosition()));
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			driveEncoder.getPosition(),
			new Rotation2d(turnEncoder.getPosition()));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Apply chassis angular offset to the desired state.
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle;

		// Optimize the reference state to avoid spinning further than 90 degrees.
		correctedDesiredState.optimize(new Rotation2d(turnEncoder.getPosition()));

		// the purpose of the condition heruender is to avoid the noise that the swerve modules make when they are idle
		if (Math.abs(correctedDesiredState.speedMetersPerSecond) < 0.001 // less than 1 mm per sec
			&& Math.abs(correctedDesiredState.angle.getRadians() - turnEncoder.getPosition()) < Rotation2d.fromDegrees(1).getRadians()) // less than 1 degree
		{
			driveMotor.set(0); // no point in doing anything
			turnMotor.set(0);
		}
		else
		{
			// Command driving and turning SPARKS MAX towards their respective setpoints.
			driveClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
			turnClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
		}

		this.desiredState = desiredState;
	}

	/** Zeroes all the SwerveModule relative encoders. */
	public void resetEncoders() {

		driveEncoder.setPosition(0); // arbitrarily set driving encoder to zero

		// temp
		//m_turningAbsoluteEncoder.resetVirtualPosition();
		// the reading and setting of the calibrated absolute turning encoder values is done in the Drivetrain's constructor

		turnMotor.set(0); // no moving during reset of relative turning encoder

		turnEncoder.setPosition(turnAbsoluteEncoder.getVirtualPosition()); // set relative position based on virtual absolute position
	}

	/** Calibrates the virtual position (i.e. sets position offset) of the absolute encoder. */
	public void calibrateVirtualPosition(double angle)
	{
		turnAbsoluteEncoder.setPositionOffset(angle);
	}

	public RelativeEncoder getDrivingEncoder()
	{
		return driveEncoder;
	}

	public RelativeEncoder getTurningEncoder()
	{
		return turnEncoder;
	}

	public AbsoluteEncoder getTurningAbsoluteEncoder()
	{
		return turnAbsoluteEncoder;
	}

	public SwerveModuleState getDesiredState()
	{
		return desiredState;
	}
}
