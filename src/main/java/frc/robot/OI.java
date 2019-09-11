package frc.robot;

import frc.robot.commands.drivetrain.DriverControl;
import frc.robot.commands.drivetrain.SplineTester;
import frc.robot.util.BBQController;

public class OI {

	BBQController driverController;
	private static OI oi;

	public static OI getInstance(){
		if (oi == null)
			oi = new OI();
		return oi;
	}

	public OI(){
		driverController = new BBQController(0);
		driverController.getAButton().whenPressed(new SplineTester());
	}

	public BBQController getDriverController() {
		return driverController;
	}

}
