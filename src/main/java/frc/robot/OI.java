package frc.robot;

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
	}

	public BBQController getDriverController() {
		return driverController;
	}

}
