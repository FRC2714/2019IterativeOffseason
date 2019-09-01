package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotMap;

public class Drivetrain extends SendableBase {

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
	}

	@Override
	public void initSendable(SendableBuilder builder) {

	}



}
