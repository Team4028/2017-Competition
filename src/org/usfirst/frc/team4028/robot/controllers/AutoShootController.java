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
	private double _visionAimingDeadband = 1.2;
	private boolean _isOnTarget;
	private boolean _isOnTargetLastCycle;
	private boolean _isShooterAtTargetSpeed;
	
	public AutoShootController(ChassisAutoAimController chassisAutoAim, RoboRealmClient roboRealm, Shooter shooter, ShooterTable shooterTable){
		_chassisAutoAim = chassisAutoAim;
		_roboRealm = roboRealm;
		_shooter = shooter;
		_shooterTable = shooterTable;
		_chassisAutoAim.setDeadband(0.5); // if under deadband, returns output of zero
		_chassisAutoAim.setMaxMinOutput(0.55, -0.55);
	}
	
	public void EnableBoilerCam() { _roboRealm.ChangeToCamera(VISION_CAMERAS.BOILER); } // Change to Boiler Camera
	
	public void EnableGearCam()   { _roboRealm.ChangeToCamera(VISION_CAMERAS.GEAR); } // Change to Gear Camera
	
	public void LoadTargetDistanceInInches(int inches) {
		//_shooterTableEntry = _shooterTable.getTelopEntryForDistance(inches);
		_shooter.CalcAutomaticShooter(inches);
	}
	
	public void LoadTargetDistanceUsingVision() {
		_shooter.CalcAutomaticShooter(_roboRealm.get_DistanceToBoilerInches());
	}
	
	public void RunShooterAtTargetSpeed() {
		_isShooterAtTargetSpeed = _shooter.ShooterMotorsReentrant(_shooterTableEntry);
	}
	
	public void StopShooter() {
		_shooter.FullShooterStop();
	}
	
	public void ToggleShooter() {
		_shooter.ToggleShooterMotors();
	}
	
	public void SetShooterVisionTargetSpeed() {
		_shooter.CalcAutomaticShooter(_roboRealm.get_DistanceToBoilerInches());
	}
	
	public void ChassisFullStop() {
		_chassisAutoAim.ChassisFullStop();
	}
	
	public void AimWithVision(double bias) {
		_chassisAutoAim.motionMagicMoveToTarget(_chassisAutoAim.currentHeading() - (_roboRealm.get_Angle()/3.5) + bias);
		if (Math.abs(_roboRealm.get_Angle()/1.5226 + bias) < _visionAimingDeadband) { // On target if under vision aiming deadband
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
		if (((System.currentTimeMillis() - _onTargetStartTime) > 600) && _isOnTarget && _isShooterAtTargetSpeed && _roboRealm.get_isVisionDataValid()) { // Ready to shoot if within deadband for longer than target time
			return true;
		} else {
			return false;
		}
	}
}