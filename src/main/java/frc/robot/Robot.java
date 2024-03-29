/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Drivetrain;


public class Robot extends TimedRobot {

	private static Drivetrain drivetrain = Drivetrain.getInstance();

	private static OI oi = OI.getInstance();

	@Override
	public void robotInit() {
	}

	@Override
	public void robotPeriodic() {

	}

	@Override
	public void autonomousInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}


	@Override
	public void testPeriodic() {
		Scheduler.getInstance().run();
	}
}
