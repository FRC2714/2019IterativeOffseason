package frc.robot.util;

public abstract class Odometer {

	protected double headingAngle;
	private double startOffset = 0;

	private double leftDistance;
	protected double leftPos;
	protected double lastLeftPos = 0;

	private double rightDistance;
	protected double rightPos;
	protected double lastRightPos = 0;

	private double hypotenuseDistance;
	protected double currentAverageVelocity;

	private double change_x, change_y;
	private double current_x, current_y;

	/**
	 * Initializes an Odometer and sets these variables
	 * @param startX
	 * @param startY
	 * @param startOffset
	 */
	public Odometer(double startX, double startY, double startOffset) {
		this.current_x = startX;
		this.current_y = startY;
		this.startOffset = startOffset;
	}

	public void reset() {
		current_x = 0;
		current_y = 0;
		headingAngle = 90;
	}

	/**
	 * Must be defined to update the odometer variables from the subsystem
	 */
	public abstract void updateEncodersAndHeading();

	/**
	 * Finds the robot's expected position using its heading and distance
	 */
	public void integratePosition() {

		updateEncodersAndHeading();

		// Get the heading
		headingAngle = headingAngle + startOffset;

		if (headingAngle > 360)
			headingAngle -= 360;

		if (headingAngle < 0)
			headingAngle += 360;

		// Get the distances
		leftDistance = (leftPos - lastLeftPos);
		rightDistance = (rightPos - lastRightPos);

		hypotenuseDistance = (leftDistance + rightDistance) / 2;

		// Calculates the expected movement
		change_x = (Math.cos(Math.toRadians(headingAngle)) * hypotenuseDistance);
		change_y = (Math.sin(Math.toRadians(headingAngle)) * hypotenuseDistance);

		current_x = current_x + change_x;
		current_y = current_y + change_y;

		lastLeftPos = leftPos;
		lastRightPos = rightPos;
	}

	/**
	 * The robot's initial offset angle in respect to 90 degrees
	 * @param offset Initial offset
	 */
	public void setOffset(double offset) {

		startOffset = offset;
	}

	public double getCurrentX() {
		return current_x;
	}

	public double getCurrentY() {
		return current_y;
	}

	public void setCurrentPosition(double inputX, double inputY) {
		current_x = inputX;
		current_y = inputY;
	}

	public double getHeadingAngle() {
		return headingAngle;
	}

	public double getCurrentAverageVelocity() {
		return currentAverageVelocity;
	}

	public void printEncoderPosition() {
		System.out.println("LE: " + leftPos + " RE: " + rightPos);
	}

	public void printOdometerPosition() {
		System.out.println("X: " + current_x + " Y: " + current_y);
	}
}