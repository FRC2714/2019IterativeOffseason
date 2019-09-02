package frc.robot.util;

import java.util.ArrayList;

public class SplineFactory {
    private double x1, x2, x3, x4, y1, y2, y3, y4, acceleration, velocity;
    private double currentFrontVelocity, currentBackVelocity;
    private boolean forwards;
    private ArrayList<MotionPose> controlPath;

    private double frontT = 0;
    private double backT = 1;

    private double tolerance = 0.00001;
    private double tStep = 0.001;
    private double period = 0.002;

    private ArrayList<Double> xValues = new ArrayList<Double>();
    private ArrayList<Double> yValues = new ArrayList<Double>();

    public SplineFactory(double period, double x1, double x2, double x3, double x4,
            double y1, double y2, double y3, double y4, double acceleration, double maxVelocity, double startVelocity,
            double endVelocity, boolean forwards) {

        this.period = period;

        this.x1 = x1;
        this.x2 = x2;
        this.x3 = x3;
        this.x4 = x4;
        this.y1 = y1;
        this.y2 = y2;
        this.y3 = y3;
        this.y4 = y4;

        this.acceleration = acceleration * period;
        this.velocity = maxVelocity * period;

        this.currentFrontVelocity = startVelocity * this.period;
        this.currentBackVelocity = endVelocity * this.period;

        this.controlPath = new ArrayList<MotionPose>();
        this.forwards = forwards;

        generate();

    }

    /**
     * 
     */
    public void generate() {

        // Fill point buffer
        int placement = 0;

        while (frontT < backT) {

            // Discrete time form trapezoidal:
            // PT = PT + VT +1/2A
            // VT = VT+A
            // where:
            // P0 and V0, are the starting position and velocities
            // PT and VT, are the position and velocity at time T
            // A is the profile acceleration

            // Ramp up velocity by acceleration
            if (currentFrontVelocity < this.velocity) {
                currentFrontVelocity += this.acceleration * this.period;
            }

            if (currentBackVelocity < this.velocity) {
                currentBackVelocity += this.acceleration * this.period;
            }

            // Find front and back position
            frontT = binaryFind(frontT, currentFrontVelocity, placement, xValues, yValues);
            backT = binaryFind(backT, -currentBackVelocity, placement + 1, xValues, yValues);
            placement++;
        }

        xValues.remove(placement);
        yValues.remove(placement);
        xValues.remove(placement - 1);
        yValues.remove(placement - 1);

        xValues.add(placement - 1, (xValues.get(placement - 1) + xValues.get(placement - 2)) / 2);
        yValues.add(placement - 1, (yValues.get(placement - 1) + yValues.get(placement - 2)) / 2);

        System.out.println("Done");

        // Calculates the MotionPoses and adds them to the array list
        for (int i = 0; i < xValues.size() - 1; i++) {

            double angle;

            double changeY = (yValues.get(i + 1) - yValues.get(i));
            double changeX = (xValues.get(i + 1) - xValues.get(i));

            if (changeY == 0) {
                if (changeX > 0) {
                    angle = 0;
                } else {
                    angle = 180;
                }
            } else if (changeX == 0) {
                if (changeY > 0) {
                    angle = 90;
                } else {
                    angle = 270;
                }

            }

            if (changeX < 0) {
                angle = Math.toDegrees(Math.atan(changeY / changeX)) + 180;
            } else if (changeY > 0) {
                angle = Math.toDegrees(Math.atan(changeY / changeX));
            } else {
                angle = Math.toDegrees(Math.atan(changeY / changeX)) + 360;
            }

            double velocity = distanceCalc(xValues.get(i + 1), xValues.get(i), yValues.get(i + 1),
                    yValues.get(i)) / period;

            if (!forwards) {
                velocity *= -1;
                angle += 180;
                if (angle > 360) {
                    angle -= 360;
                }
            }

            controlPath.add(new MotionPose(angle, velocity, xValues.get(i), yValues.get(i)));

        }

    }

    /**
     * 
     * @return returns the generated list of MotionPose objects.
     */
    public ArrayList<MotionPose> getSpline() {
        return controlPath;
    }

    /**
     * Finds the point at a given distance
     * @param startT Value of T the search starts at
     * @param distance Desired distance
     * @param location Location for point insertion
     * @param xValues List of x points being created
     * @param yValues List of y points being created
     * @return Resultant T value
     */
    public double binaryFind(double startT, double distance, int location, ArrayList<Double> xValues, ArrayList<Double> yValues) {
        double internalT = startT;
        double tStep_modified = this.tStep;
        double inverted = 1;
        double direction = 1;
        double lastDirection = 1;
        if (distance < 0) {
            inverted = -1;
            direction = -1;
            lastDirection = -1;
        }

        double startX = quarticCalc(internalT, this.x1, this.x2, this.x3, this.x4);
        double startY = quarticCalc(internalT, this.y1, this.y2, this.y3, this.y4);
        double newX, newY;

        double currentDistance = 0;
        double targetDistance = Math.abs(distance);
        double distanceDelta = targetDistance - currentDistance;

        do {

            if (distanceDelta * inverted < 0) {
                // Past target
                direction = -1;
                if (direction != lastDirection) {
                    tStep_modified *= 0.5;
                }
            } else {
                // Not past target
                direction = 1;
                if (direction != lastDirection) {
                    tStep_modified *= 0.5;
                }
            }

            internalT += tStep_modified * direction;

            newX = quarticCalc(internalT, this.x1, this.x2, this.x3, this.x4);
            newY = quarticCalc(internalT, this.y1, this.y2, this.y3, this.y4);

            currentDistance = distanceCalc(startX, newX, startY, newY);

            lastDirection = direction;
            distanceDelta = targetDistance - currentDistance;

        } while (Math.abs(distanceDelta) > this.tolerance);

        this.xValues.add(location, newX);
        this.yValues.add(location, newY);

        return internalT;
    }

    /**
     * Quartic spline equation with 4 control points
     * @param t T step input
     * @param c1
     * @param c2
     * @param c3
     * @param c4
     * @return
     */
    public double quarticCalc(double t, double c1, double c2, double c3, double c4) {
        return (Math.pow(1 - t, 3) * c1) + 3 * (Math.pow(1 - t, 2) * t * c2) + 3 * (Math.pow(t, 2) * (1 - t) * c3)
                + (Math.pow(t, 3) * c4);
    }

    /**
     * Calculate distance between points
     * @param x1
     * @param x2
     * @param y1
     * @param y2
     * @return
     */
    public double distanceCalc(double x1, double x2, double y1, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }
}