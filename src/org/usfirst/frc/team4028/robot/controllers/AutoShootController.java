package org.usfirst.frc.team4028.robot.controllers;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.VISION_CAMERAS;
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
	private long _onTargetStartTime;
	private double _visionAimingDeadband = 1.3;
	private boolean _isShooterAtTargetSpeed;
	private boolean _isOnTarget;
	private boolean _isOnTargetLastCycle;
	private boolean _readyToShoot;
	
	public AutoShootController(ChassisAutoAimController chassisAutoAim, RoboRealmClient roboRealm, Shooter shooter, ShooterTable shooterTable){
		_chassisAutoAim = chassisAutoAim;
		_roboRealm = roboRealm;
		_shooter = shooter;
		_shooterTable = shooterTable;
		_chassisAutoAim.setDeadband(0.5);
		_chassisAutoAim.setMaxMinOutput(0.55, -0.55);
	}
	
	public void EnableBoilerCam() { _roboRealm.ChangeToCamera(VISION_CAMERAS.BOILER); }
	
	public void EnableGearCam()   { _roboRealm.ChangeToCamera(VISION_CAMERAS.GEAR); }
	
	public void LoadTargetDistanceInInches(int inches) {
		_shooterTableEntry = _shooterTable.getAutonEntryForDistance(inches);
	}
	
	public void RunShooterAtTargetSpeed() {
		_isShooterAtTargetSpeed = _shooter.ShooterMotorsReentrant(_shooterTableEntry);
	}
	
	public void InitializeVisionAiming() {
		_chassisAutoAim.zeroTotalError();
		_chassisAutoAim.loadNewVisionTarget(_roboRealm.get_Angle()/1.5226);
	}
	
	public void AimWithVision() {
		_chassisAutoAim.loadNewVisionTarget(_roboRealm.get_Angle()/1.5226);
		_chassisAutoAim.update();
		if (Math.abs(_roboRealm.get_Angle()/1.5226) < _visionAimingDeadband) {
			_isOnTarget = true;
		} else {
			_isOnTarget = false;
		}
		
		if (_isOnTarget && !_isOnTargetLastCycle) {
			_onTargetStartTime = System.currentTimeMillis();
		}
		
		_isOnTargetLastCycle = _isOnTarget;
	}
	
	public boolean IsReadyToShoot() {
		if (((System.currentTimeMillis() - _onTargetStartTime) > 1000) && _isOnTarget) {
			return true;
		} else {
			return false;
		}
	}
}