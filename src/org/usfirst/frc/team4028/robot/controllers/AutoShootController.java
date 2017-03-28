package org.usfirst.frc.team4028.robot.controllers;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ViSION_CAMERAS;
import org.usfirst.frc.team4028.robot.sensors.RoboRealmClient;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;
import org.usfirst.frc.team4028.robot.utilities.ShooterTable;
import org.usfirst.frc.team4028.robot.utilities.ShooterTableEntry;

public class AutoShootController {
	ChassisAutoAimController _chassisAutoAim;
	Shooter _shooter;
	ShooterTable _shooterTable;
	ShooterTableEntry _shooterTableEntry;
	RoboRealmClient _roboRealm;
	double _visionTurnError;
	double _visionAimingDeadband;
	
	public AutoShootController(ChassisAutoAimController chassisAutoAim, RoboRealmClient roboRealm, Shooter shooter, ShooterTable shooterTable){
		_chassisAutoAim = chassisAutoAim;
		_roboRealm = roboRealm;
		_shooter = shooter;
		_shooterTable = shooterTable;
		_chassisAutoAim.setDeadband(0.5);
	}
	
	public void EnableBoilerCam() {
			_roboRealm.ChangeToCamera(ViSION_CAMERAS.BOILER);
	}
	
	public void EnableGearCam() {
			_roboRealm.ChangeToCamera(ViSION_CAMERAS.GEAR);
	}
	
	public void LoadTargetDistanceInInches(int inches) {
		_shooterTableEntry = _shooterTable.getEntryForDistance(inches);
	}
	
	public void RunShooterAtTargetSpeed() {
		_shooter.ShooterMotorsReentrant(_shooterTableEntry);
	}
	
	public void InitializeVisionAiming() {
		_chassisAutoAim.zeroTotalError();
		_chassisAutoAim.loadNewVisionTarget(_roboRealm.get_Angle()/1.5226);
	}
	
	public void AimAndShootWhenReady() {
		_chassisAutoAim.loadNewVisionTarget(_roboRealm.get_Angle()/1.5226);
		_chassisAutoAim.update();
		//TODO: check magnitude of error, if under threshhold, shoot ball, else turn chassis
	}
}
