package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.frc2.command.SendableCommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriverControl extends SendableCommandBase {

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

		if (Math.abs(controlsProcessor.getLeftJoystick()) > .15)
			power = controlsProcessor.getLeftJoystick();
		if (Math.abs(controlsProcessor.getRightJoystick()) > .15)
			pivot = controlsProcessor.getRightJoystick();

		arcadeDrive(-power, pivot, 0.04, 0.08);
		// System.out.println("Right Encoder: " + rightShaftEncoder.getDistance() + "\tLeft Encoder: " + leftShaftEncoder.getDistance());
		// System.out.println("X = " + odometer.getCurrentX() + "|| Y = " + odometer.getCurrentY());

		// System.out.println("Odometer heading angle " + odometer.getHeadingAngle());
	}

	@Override
	public boolean isFinished() {
		return !driverControlled;
	}

	@Override
	public void end(boolean interrupted) {
		Drivetrain.getInstance().closedLoopArcade(0, 0);
	}


}
