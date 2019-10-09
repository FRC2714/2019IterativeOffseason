package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


public class BBQController {

	private XboxController xbox;


	private Button aButton;
	private Button bButton;
	private Button xButton;
	private Button yButton;
	private Button leftBumper;
	private Button rightBumper;
	private Button backButton;
	private Button startButton;

	public BBQController(int port) {

		xbox = new XboxController(port);

		aButton = new JoystickButton(xbox, 1);
		bButton = new JoystickButton(xbox, 2);
		xButton = new JoystickButton(xbox, 3);
		yButton = new JoystickButton(xbox, 4);
		leftBumper = new JoystickButton(xbox, 5);
		rightBumper = new JoystickButton(xbox, 6);
		backButton = new JoystickButton(xbox, 7);
		startButton = new JoystickButton(xbox, 8);

	}


	public double getYAxis(GenericHID.Hand hand) {
		return xbox.getY(hand);
	}

	public double getXAxis(GenericHID.Hand hand) {
		return xbox.getX(hand);
	}


	public double getTrigger(GenericHID.Hand hand) {
		return xbox.getTriggerAxis(hand);
	}

	public Button getAButton() {
		return aButton;
	}

	public Button getBButton() {
		return bButton;
	}

	public Button getXButton() {
		return xButton;
	}

	public Button getYButton() {
		return yButton;
	}

	public Button getLeftBumper() {
		return leftBumper;
	}

	public Boolean getLeftBumperBoolean() {
		return leftBumper.get();
	}

	public Button getRightBumper() {
		return rightBumper;
	}

	public Boolean getRightBumperBoolean() {
		return rightBumper.get();
	}

	public Button getBackButton() {
		return backButton;
	}

	public Button getStartButton() {
		return startButton;
	}

	public void setRumble(double intensity) {
		xbox.setRumble(GenericHID.RumbleType.kLeftRumble, intensity);
		xbox.setRumble(GenericHID.RumbleType.kRightRumble, intensity);

	}

}
