package org.usfirst.frc.team4028.robot.controllers;

import org.usfirst.frc.team4028.robot.util.PIDCalculator;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.DriverStation;

public class ChassisAutoAimController {
	PIDCalculator _autoAimPID;
	Chassis _chassis;
	NavXGyro _navX;
	double _kp = 0.11;
	double _ki = 0.0;
	double _kd = 0.0;
	double _setError = 0.0;
	
	public ChassisAutoAimController(Chassis chassis, NavXGyro navX) {
		_chassis = chassis;
		_navX = navX;
		_autoAimPID = new PIDCalculator(_kp, _ki, _kd);
	}
	
	public boolean onTarget() {
		return _autoAimPID.onTarget(); // Check if the PID loop error is within a set deadband
	}
	
	public void loadNewTarget(double angle) {
		_autoAimPID.reset();			// Reset the PID Calculator
		_autoAimPID.setSetpoint(angle); // Set a new target angle
		DriverStation.reportError("New Setpoint Loaded", false);
	}
	
	public void loadNewVisionTarget(double angle) {
		_autoAimPID.reset();
		_autoAimPID.setSetpoint(_navX.getYaw() + angle);
	}
	
	public void update() {
		double motorOutput = _autoAimPID.calculate(_navX.getYaw()); // Pass in current angle to calculate motor output
		_chassis.TankDrive(-motorOutput, motorOutput);
	}
	
	public void updateVision(double currentError) {
		if (_setError != currentError) {
			_setError = currentError;
			_autoAimPID.setSetpoint(_navX.getYaw() + _setError);
		}
		
		double motorOutput = _autoAimPID.calculate(_navX.getYaw());
		_chassis.TankDrive(-motorOutput, motorOutput);
	}
	
	public void zeroVisionError() {
		_setError = 0.0;
	}
}