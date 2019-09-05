package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriverControl extends CommandBase {

	Drivetrain drivesystem;

	public DriverControl(Drivetrain drivetrain){
		drivesystem = drivetrain;
		addRequirements(drivesystem);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		double power = 0;
		double pivot = 0;

		double yAxisLeft = OI.getInstance().getDriverController().getYAxis(GenericHID.Hand.kLeft);
		double xAxisRight = OI.getInstance().getDriverController().getXAxis(GenericHID.Hand.kRight);

		if (Math.abs(yAxisLeft) > .15)
			power = yAxisLeft;
		if (Math.abs(xAxisRight) > .15)
			pivot = xAxisRight;

		drivesystem.arcadeDrive(-power, pivot, 0.04, 0.08);
		// System.out.println("Right Encoder: " + rightShaftEncoder.getDistance() + "\tLeft Encoder: " + leftShaftEncoder.getDistance());
		// System.out.println("X = " + odometer.getCurrentX() + "|| Y = " + odometer.getCurrentY());

		// System.out.println("Odometer heading angle " + odometer.getHeadingAngle());
	}

	@Override
	public boolean isFinished() {
		return false; //return !driverControlled?
	}

	@Override
	public void end(boolean interrupted) {
		Drivetrain.getInstance().closedLoopArcade(0, 0);
	}


}
