package frc.robot.util;

public abstract class PositionController {
	double posP, posI, posD, velP, velI, velD, velF;

	PID positionPID = new PID(posP, posI, posD);

	protected double currentPosition, setpoint;

	/**
	 * Initializes PositionController and sets PID values
	 * @param posP Proportional
	 * @param posI Integral
	 * @param posD Derivative
	 */
	public PositionController(double posP, double posI, double posD) {

		this.posP = posP;
		this.posI = posI;
		this.posD = posD;

	}

	/**
	 * This must be defined to update the current values from the subsystem
	 */
	public abstract void updateValues();

	/**
	 * This must be defined to update the output to the subsystem
	 * @param output The PID output to set
	 */
	public abstract void setOutput(double output);

	/**
	 * Removes the I term and resets the controller
	 * Can be overridden for other control systems
	 */
	public void reset() {
		positionPID.reset();
	}

	/**
	 * Configures the target value
	 * @param setpoint Target value
	 */
	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}

	/**
	 * This will be called inside the run function of the parent subsystem
	 */
	public void runController() {
		updateValues();
		double output = 0;

		// Perform necessary control function here
		output = positionPID.getOutput(this.currentPosition, this.setpoint);

		setOutput(output);
	}

}
