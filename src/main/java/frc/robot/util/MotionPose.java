package frc.robot.util;

public class MotionPose {

    public double angle, velocity, x, y;

    /**
     * Creates a MotionPose with this reference frame
     * @param angle
     * @param velocity
     * @param x
     * @param y
     */
    public MotionPose(double angle, double velocity, double x, double y) {
        this.angle = angle;
        this.velocity = velocity;
        this.x = x;
        this.y = y;
    }

    /**
     * Calculates the robot's current perpendicular error
     * @param currentX
     * @param currentY
     * @return Current perpendicular error
     */
    public double getOrthogonalDisplacement(double currentX, double currentY) {
        double errorY = currentY - y;
        double errorX = currentX - x;
        double unitX = Math.cos(Math.toRadians(angle + 90));
        double unitY = Math.sin(Math.toRadians(angle + 90));
        double dotProduct = (unitX * errorX) + (unitY * errorY);

        return dotProduct;
    }

    /**
     * Calculates the robot's current tangential error
     * @param currentX
     * @param currentY
     * @return Current tangential error
     */
    public double getTangentialDisplacement(double currentX, double currentY) {
        double errorY = currentY - y;
        double errorX = currentX - x;
        double unitX = Math.cos(Math.toRadians(angle));
        double unitY = Math.sin(Math.toRadians(angle));
        double dotProduct = (unitX * errorX) + (unitY * errorY);

        return dotProduct;
    }
    
    /**
     * Calculates the robot's current angular error
     * @param currentAngle
     * @return Current angular error
     */
	public double getAngularDisplacement(double currentAngle) {
        double angularError = currentAngle - angle;
        
        while (angularError >= 180.0) {
			angularError -= 360.0;
		}
		while (angularError < -180.0) {
			angularError += 360.0;
        }
        
		return angularError;
	}

    // Calculate distance
    public double distanceCalc(double x1, double x2, double y1, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    public String toString() {
        return "X = " + x + " Y = " + y + " Angle = " + angle + " Vel = " + velocity;
    }
}