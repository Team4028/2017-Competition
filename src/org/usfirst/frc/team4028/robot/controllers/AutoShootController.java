package org.usfirst.frc.team4028.robot.controllers;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ViSION_CAMERAS;
import org.usfirst.frc.team4028.robot.sensors.RoboRealmClient;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoShootController {
	ChassisAutoAimController _chassisAutoAim;
	Shooter _shooter;
	RoboRealmClient _roboRealm;
	double _visionTurnError;
	
	public AutoShootController(ChassisAutoAimController chassisAutoAim, RoboRealmClient roboRealm, Shooter shooter){
		_chassisAutoAim = chassisAutoAim;
		_roboRealm = roboRealm;
		_shooter = shooter;
		_chassisAutoAim.setDeadband(2.0);
	}
	
	public void EnableBoilerCam() {
			_roboRealm.ChangeToCamera(ViSION_CAMERAS.BOILER);
	}
	
	public void EnableGearCam() {
			_roboRealm.ChangeToCamera(ViSION_CAMERAS.GEAR);
	}
	
	public void Initialize() {
	}
	
	public void AimAndShootWhenReady() {
		DriverStation.reportError("Trying to Aim: " + _roboRealm.get_isVisionDataValid(), false);
		if (_roboRealm.get_isVisionDataValid()) {
			_chassisAutoAim.updateVision(_roboRealm.get_Angle());
		}
		
		//TODO: check magnitude of error, if under threshhold, shoot ball, else turn chassis
	}
}
