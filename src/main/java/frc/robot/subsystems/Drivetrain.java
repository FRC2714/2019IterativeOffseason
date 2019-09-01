package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.frc2.command.Command;
import edu.wpi.first.wpilibj.frc2.command.SendableSubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotMap;
import frc.robot.util.DrivingController;
import frc.robot.util.Odometer;

public class Drivetrain extends SendableSubsystemBase {

	// Drivetrain motors
	private CANSparkMax lMotor0 = new CANSparkMax(1, MotorType.kBrushless);
	private CANSparkMax lMotor1 = new CANSparkMax(2, MotorType.kBrushless);
	private CANSparkMax lMotor2 = new CANSparkMax(3, MotorType.kBrushless);
	private CANSparkMax rMotor0 = new CANSparkMax(4, MotorType.kBrushless);
	private CANSparkMax rMotor1 = new CANSparkMax(5, MotorType.kBrushless);
	private CANSparkMax rMotor2 = new CANSparkMax(6, MotorType.kBrushless);

	// PID controllers
	private CANPIDController lPidController = lMotor0.getPIDController();
	private CANPIDController rPidController = rMotor0.getPIDController();

	// Differential drivetrain
	private DifferentialDrive drive = new DifferentialDrive(lMotor0, rMotor0);

	// PID coefficients
	private final double kMinOutput = -1;
	private final double kMaxOutput = 1;

	private final double kP = 4.8e-5;
	private final double kI = 5.0e-7;
	private final double kD = 0.0;
	private final double kIS = 0.0;

	private final double lKFF = 1.77e-4;
	private final double rKFF = 1.78e-4;

	private final double rpmToFeet = 0.003135; // Convert RPM to ft/s

	private final double sensitivity = 2.5;
	private final double maxVelocity = 13;

	private double lastVelocity = 0;

	// Ramp code
	private double currentOpenArcadePower;

	private boolean driverControlled = false;

	// Gearbox encoders
	private Encoder leftShaftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, true, CounterBase.EncodingType.k4X);
	private Encoder rightShaftEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, true,
			CounterBase.EncodingType.k4X);

	// NavX gyro
	private AHRS navX = new AHRS(SPI.Port.kMXP);

	//limelight
	NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

	public Drivetrain(){

		drive.setSafetyEnabled(false);
		// Configure follow mode
		lMotor1.follow(lMotor0);
		lMotor2.follow(lMotor0);
		rMotor1.follow(rMotor0);
		rMotor2.follow(rMotor0);

		// Setup up PID coefficients
		lPidController.setP(kP);
		lPidController.setI(kI);
		lPidController.setD(kD);
		lPidController.setIZone(kIS);
		lPidController.setFF(lKFF);
		lPidController.setOutputRange(kMinOutput, kMaxOutput);

		rPidController.setP(kP);
		rPidController.setI(kI);
		rPidController.setD(kD);
		rPidController.setIZone(kIS);
		rPidController.setFF(rKFF);
		rPidController.setOutputRange(kMinOutput, kMaxOutput);

		lMotor0.enableVoltageCompensation(12.0);
		rMotor0.enableVoltageCompensation(12.0);

		lMotor0.setSmartCurrentLimit(50);
		lMotor1.setSmartCurrentLimit(50);
		lMotor2.setSmartCurrentLimit(50);

		rMotor0.setSmartCurrentLimit(50);
		rMotor1.setSmartCurrentLimit(50);
		rMotor2.setSmartCurrentLimit(50);

		System.out.println("resetting");
		navX.reset();
		navX.zeroYaw();

		odometer.reset();

		currentOpenArcadePower = 0;

		// leftEncoder.setDistancePerPulse(-0.0495);
		// rightEncoder.setDistancePerPulse(0.00105);
		leftShaftEncoder.reset();
		rightShaftEncoder.reset();
		leftShaftEncoder.setDistancePerPulse(0.0007819);
		rightShaftEncoder.setDistancePerPulse(0.00078012);

		lMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);

		drivingController.clearControlPath();
	}

	// Instantiate odometer and link in encoders and navX
	public Odometer odometer = new Odometer(0,0,0) {

		@Override
		public void updateEncodersAndHeading() {
			this.headingAngle = -navX.getYaw() + 90;
			if(this.headingAngle < 0) {
				this.headingAngle += 360;
			}

			this.leftPos = leftShaftEncoder.getDistance();
			this.rightPos = rightShaftEncoder.getDistance();

			double leftVelocity = leftShaftEncoder.getRate();
			double rightVelocity = rightShaftEncoder.getRate();

			this.currentAverageVelocity = (leftVelocity + rightVelocity) / 2;
		}
	};

	// Instantiate point controller for autonomous driving
	public DrivingController drivingController = new DrivingController(0.01) {

		/**
		 * Use output from odometer and pass into autonomous driving controller
		 */
		@Override
		public void updateVariables(){
			this.currentX = odometer.getCurrentX();
			this.currentY = odometer.getCurrentY();
			this.currentAngle = odometer.getHeadingAngle();
			this.currentAverageVelocity = odometer.getCurrentAverageVelocity();
		}

		/**
		 * Link autonomous driving controller to the drive train motor control
		 */
		@Override
		public void driveRobot(double power, double pivot) {
			closedLoopArcade(power, pivot);
		}
	};

	public void destruct() {
		driverControlled = false;

		lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);

		lMotor0.set(0);
		rMotor0.set(0);
	}

	// General arcade drive
	public void arcadeDrive(double power, double pivot) {
		drive.arcadeDrive(power, pivot);
	}

	public void arcadeDrive(double power, double pivot, double rampUp, double rampDown) {
		int currentDirection = (int)(Math.abs(currentOpenArcadePower) / currentOpenArcadePower);
		int desiredDirection = (int)(Math.abs(power) / power);

		if (currentDirection * desiredDirection > 0) {
			rampCalc(power, rampUp);
		} else {
			rampCalc(power, rampDown);
		}

		// System.out.println("Current Arcade Power: " + currentOpenArcadePower + "\tCurrent Arcade Pivot: " + currentOpenArcadePivot);
		arcadeDrive(currentOpenArcadePower, pivot);
	}

	private void rampCalc(double power, double rampUp) {
		if(currentOpenArcadePower < power) {
			currentOpenArcadePower += rampUp;

			if(currentOpenArcadePower > power) { currentOpenArcadePower = power; }
		}
		else if(currentOpenArcadePower > power) {
			currentOpenArcadePower -= rampUp;

			if(currentOpenArcadePower < power) { currentOpenArcadePower = power; }
		}
	}

	// Closed loop velocity based tank without an acceleration limit
	public void closedLoopTank(double leftVelocity, double rightVelocity) {
		lPidController.setReference(leftVelocity / rpmToFeet, ControlType.kVelocity);
		rPidController.setReference(-rightVelocity / rpmToFeet, ControlType.kVelocity);
		// System.out.println("ls: " + leftVelocity / rpmToFeet + " rs: " + -rightVelocity / rpmToFeet);
	}

	// Closed loop arcade based tank
	public void closedLoopArcade(double velocity, double pivot) {
		pivot = pivot * sensitivity;
		closedLoopTank(velocity - pivot, velocity + pivot);
		// System.out.println("pivot " + pivot);
	}

	// Closed loop velocity based tank with an acceleration limit
	public void closedLoopArcade(double velocity, double pivot, double accelLimit) {
		accelLimit *= 0.02;

		double velocitySetpoint = velocity;

		if (Math.abs(velocity - lastVelocity) > accelLimit) {
			if (velocity - lastVelocity > 0)
				velocitySetpoint = lastVelocity + accelLimit;
			else
				velocitySetpoint = lastVelocity - accelLimit;
		}

		closedLoopArcade(velocitySetpoint, pivot);

		lastVelocity = velocitySetpoint;
		System.out.println("velocity setpoint: " + velocitySetpoint);
	}

	// Output encoder values
	public void getEncoderValues() {
		System.out.println("LE: " + leftShaftEncoder.getDistance() + " RE: " + rightShaftEncoder.getDistance());
	}

	public double getMaxVelocity(){
		return maxVelocity;
	}

	public void addForwardSpline(double xInitial, double yInitial, double thetaInitial, double lInitial,
	                             double xFinal, double yFinal, double thetaFinal, double lFinal, double maxAcceleration,
	                             double maxVelocity, double startVelocity, double endVelocity) {

		thetaInitial = Math.toRadians(thetaInitial);
		thetaFinal = Math.toRadians(thetaFinal);

		double x2 = lInitial * Math.cos(thetaInitial) + xInitial;
		double x3 = lFinal * Math.cos(thetaFinal + Math.PI) + xFinal;
		double y2 = lInitial * Math.sin(thetaInitial) + yInitial;
		double y3 = lFinal * Math.sin(thetaFinal + Math.PI) + yFinal;

		System.out.println("Forward Spline Generating");

		drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
				maxAcceleration, maxVelocity, startVelocity, endVelocity, true);
	}

	public void addBackwardsSpline(double xInitial, double yInitial, double thetaInitial, double lInitial,
	                               double xFinal, double yFinal, double thetaFinal, double lFinal, double maxAcceleration,
	                               double maxVelocity, double startVelocity, double endVelocity) {

		thetaInitial = Math.toRadians(thetaInitial);
		thetaFinal = Math.toRadians(thetaFinal);

		double x2 = lInitial * Math.cos(thetaInitial + Math.PI) + xInitial;
		double x3 = lFinal * Math.cos(thetaFinal) + xFinal;
		double y2 = lInitial * Math.sin(thetaInitial + Math.PI) + yInitial;
		double y3 = lFinal * Math.sin(thetaFinal) + yFinal;

		System.out.println("Backwards Spline Generating");

		drivingController.addSpline(xInitial, x2, x3, xFinal, yInitial, y2, y3, yFinal,
				maxAcceleration, maxVelocity, startVelocity, endVelocity, false);
	}

	@Override
	public void setDefaultCommand(Command defaultCommand) {

	}
}
