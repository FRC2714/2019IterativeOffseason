package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.StateMachine;
import frc.robot.subsystems.Drivetrain;

public class SplineTester extends Command {

	private Drivetrain drivetrainInstance = Drivetrain.getInstance();

	public SplineTester(){

	}

	@Override
	protected void initialize() {
//		drivetrainInstance.odometer.setOffset(-180);
//		drivetrainInstance.addForwardSpline(0,0,270,3,0,9,270,3,5,10,0,0);
		drivetrainInstance.addForwardSpline(0,0,90,3,0,9,90,3,5,10,0,0);
		drivetrainInstance.motionControl = StateMachine.drivetrainState.PATHTRACKING;
	}

	@Override
	protected boolean isFinished() {
		System.out.println("Path Tracking isFinished");
		return drivetrainInstance.drivingController.isFinished();
	}


	@Override
	protected void end() {
		drivetrainInstance.motionControl = StateMachine.drivetrainState.DRIVERCONTROL;
		System.out.println("X = " + Drivetrain.getInstance().odometer.getCurrentX() + "|| Y = " + Drivetrain.getInstance().odometer.getCurrentY());
	}
}
