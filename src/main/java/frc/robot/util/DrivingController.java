package frc.robot.util;

import java.util.ArrayList;

public abstract class DrivingController {

	/**
	 * Controls the magnitude of angular correction
	 * Corrects both the anglular and perpendicular error
	 */
	// Samson control was originally at 0.25 * 0.6
	private PID samsonControl = new PID(0.06, 0.0001, 0.0);
	private double samsonOutput;

	/**
	 * Controls the magnitude of tangential correction
	 */
	private PID tangentialControl = new PID(0.4, 0.0, 0.0);
	private double tangentialOutput;

	/**
	 * k2: For perpendicular error
	 * k3: For angular error
	 */
	private double k2 = 1.0;
	private double k3 = 1.0;

	/**
	 * Populates an array list from the SplineFactory
	 */
	private ArrayList<MotionPose> controlPath = new ArrayList<MotionPose>();
	private int iterator = 0;

	protected double currentX;
	protected double currentY;
	protected double currentAngle;
	protected double currentAverageVelocity;

	private boolean pathFinished = false;

	/**
	 * Time separation between points in the controlPath in seconds
	 */
	private double period;

	/**
	 * Initializes DrivingController and sets the period
	 * @param period In seconds
	 */
	public DrivingController(double period) {
		this.period = period;

		this.samsonControl.setMaxIOutput(0.15);
	}


	/**
	 * Run function for Driving Controller uses distance and angle controllers
	 */
	public void run() {
		// Test
		// System.out.println(System.nanoTime());

		// Update using abstracted functions from the calling class
		updateVariables();

		// Move to the next point in the spline
		if(iterator < controlPath.size() - 1) {
			this.iterator++;
		}
		else {
			pathFinished = true;
		}

		// Use tangential correction and velocity control cascaded to control velocity and position.
		double orthogonalError = controlPath.get(iterator).getOrthogonalDisplacement(currentX, currentY);
		double tangentialError = controlPath.get(iterator).getTangentialDisplacement(currentX, currentY);
		double angularError = controlPath.get(iterator).getAngularDisplacement(currentAngle);

		double refVelocity = controlPath.get(iterator).velocity;

		double samsonCorrection2;


		if (angularError > 1) {
			samsonCorrection2 = (k2 * orthogonalError * refVelocity) / angularError;
		} else {
			samsonCorrection2 = (k2 * orthogonalError * refVelocity) / 1;
		}
		
		double samsonCorrection3 = k3 * angularError;

		double samsonSum = samsonCorrection2 + samsonCorrection3;

		samsonOutput = samsonControl.getOutput(samsonSum, 0);
		tangentialOutput = tangentialControl.getOutput(tangentialError, 0);

		// System.out.println("Ref Velocity: " + refVelocity);

		// Both +
		driveRobot(refVelocity + tangentialOutput, samsonOutput);
		// System.out.println(samsonOutput);

	}

	// Abstract functions to move and get position of the robot
	public abstract void updateVariables();
	public abstract void driveRobot(double power, double pivot);

	/**
	 * Creates the cubic spline between these points
	 * @param x1
	 * @param x2
	 * @param x3
	 * @param x4
	 * @param y1
	 * @param y2
	 * @param y3
	 * @param y4
	 * @param acceleration Designates the acceleration of the robot along the path
	 * @param maxVelocity Designates the max velocity of the path
	 * @param startVelocity Designates the velocity at the start of the path
	 * @param endVelocity Designates the velocity at the end of the path
	 * @param forwards Designates if the robot drives forwards or backwards along the path
	 */
	public void addSpline(double x1, double x2, double x3, double x4, double y1, double y2, double y3, double y4,
			double acceleration, double maxVelocity, double startVelocity, double endVelocity, boolean forwards) {
		
		SplineFactory nextSpline = new SplineFactory(this.period, x1, x2, x3, x4, y1, y2, y3, y4, acceleration, maxVelocity,
			startVelocity, endVelocity, forwards);
		System.out.println("Forwards : " + forwards);
	
		controlPath.addAll(nextSpline.getSpline());

		// for (MotionPose i : controlPath) {
		// 	System.out.println("Velocity: " + i.velocity);
		// }

	}

	public double getAngleValues(){
		return currentAngle;
		
	}

	/**
	 * Move to next motion pose in the sequence
	 */
	public void next() {
		if(iterator < controlPath.size()) { this.iterator++; }
	}

	public int getIterator() {
		return iterator;
	}

	public void clearControlPath(){
		controlPath.clear();
	}

	public ArrayList<MotionPose> getControlPath(){
		return controlPath;
	}

	/**
	 * 
	 * @return
	 */
	public boolean isFinished() {
		return pathFinished;
	}

	public void setIsFinished(boolean isFinished) {
		pathFinished = isFinished;
	}
}