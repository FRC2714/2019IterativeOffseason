package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.StateMachine;
import frc.robot.subsystems.Drivetrain;

public class DriverControl extends Command {

	private Drivetrain drivetrainInstance = Drivetrain.getInstance();

	public DriverControl(){
		requires(drivetrainInstance);
	}

	@Override
	public void initialize() {
		System.out.println("Initialized Driver Control");
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

		if(drivetrainInstance.motionControl == StateMachine.drivetrainState.DRIVERCONTROL)
			Drivetrain.getInstance().arcadeDrive(-power, pivot, 0.04, 0.08);
//		 System.out.println("Right Encoder: " + Drivetrain.getInstance().rightShaftEncoder.getDistance() + "\tLeft Encoder: " + Drivetrain.getInstance().leftShaftEncoder.getDistance());
//		 System.out.println("X = " + Drivetrain.getInstance().odometer.getCurrentX() + "|| Y = " + Drivetrain.getInstance().odometer.getCurrentY());

//		 System.out.println("Odometer heading angle " + drivetrainInstance.odometer.getHeadingAngle());
	}

	@Override
	public boolean isFinished() {
		return false; //return !driverControlled?
	}

	@Override
	protected void end() {
		Drivetrain.getInstance().closedLoopArcade(0, 0);
	}


}
