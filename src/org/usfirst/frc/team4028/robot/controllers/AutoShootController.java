package org.usfirst.frc.team4028.robot.controllers;

import org.usfirst.frc.team4028.robot.subsystems.Shooter;

public class AutoShootController {
	ChassisAutoAimController _chassisAutoAim;
	Shooter _shooter;
	double _visionTurnError;
	
	public AutoShootController(ChassisAutoAimController chassisAutoAim, Shooter shooter){
		_chassisAutoAim = chassisAutoAim;
		_shooter = shooter;
	}
	
	public void initialize() {
		//_visionTurnError = 
		_chassisAutoAim.loadNewVisionTarget(_visionTurnError);
	}
	
	public void update() {
		_chassisAutoAim.update();
	}
}
