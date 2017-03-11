package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.ChassisAutoAimController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.robot.util.TurnAndShootTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Turn and shoot into the Boiler" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	25.Feb.2017	Initial Version
//1.0 		Sebas 		6.Mar.2017	Added Motion Profile
//------------------------------------------------------
//=====> For Changes see Sebas
public class TurnAndShoot {
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private Shooter _shooter;
	private Chassis _chassis;
	private NavXGyro _navX;
	private TrajectoryDriveController _trajController;
	private ChassisAutoAimController _autoAim;
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	// define class level constants
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public TurnAndShoot(GearHandler gearHandler, Chassis chassis, NavXGyro navX, Shooter shooter, TrajectoryDriveController trajController) {
		// these are the subsystems that this auton routine needs to control
		_gearHandler = gearHandler;
		_shooter = shooter;
		_chassis = chassis;
		_navX = navX;
		_autoAim = new ChassisAutoAimController(_chassis, _navX);
		_trajController = trajController;
		DriverStation.reportError("Auton Initialized", false);
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	// execute any logic to initialize this object before ExecuteRentrant is called
	public void Initialize() {
		_autonStartedTimeStamp = System.currentTimeMillis();
		_isStillRunning = true;
		
		_autoAim.loadNewTarget(-45.0);
		_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
		_trajController.configureIsHighGear(false);
		_trajController.loadProfile(MOTION_PROFILE.TURN_AND_SHOOT, false);
		_trajController.enable();
		DriverStation.reportWarning("===== Entering TurnAndShoot Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() {
		// =======================================
		// if not complete, this must run concurrently with all auton routines
		// =======================================
      	if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      		//			we must treat it as a Reentrant function
      		//			and automatically recall it until complete
    		_gearHandler.ZeroGearTiltAxisReentrant();
    	}
 
		if(_trajController.onTarget()) {
			_trajController.disable();
			_autoAim.update();
			if(_autoAim.onTarget()) {
				DriverStation.reportError("PEW PEW PEW PEW PEW", false);
			}
		}
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportWarning("===== Completed TurnAndShoot Auton =====", false);
		}
		
		return _isStillRunning; 
	}
	
	public void Disabled() {
		_trajController.disable();
	}
	
	//============================================================================================
	// Properties follow
	//============================================================================================
	public boolean getIsStillRunning() {
		return _isStillRunning;
	}
}
