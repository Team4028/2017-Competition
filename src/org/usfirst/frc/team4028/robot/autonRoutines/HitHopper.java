package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;
import org.usfirst.frc.team4028.robot.util.MoveToHopperTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

// this class implements the logic for the "Hit the Hopper and Shoot" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	7.Mar.2017	Initial Version
//------------------------------------------------------
//=====> For Changes see Sebas
public class HitHopper {
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private Shooter _shooter;
	private TrajectoryDriveController _trajController;
	
	private enum AUTON_STATE {
		UNDEFINED,
		MOVE_TO_BOILER_HELLA_FAST,
		WAIT,
		MOVE_TO_SHOOTING_POSITION,
		SHOOT
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private long _waitStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	// define class level constants
	private static final int WAIT_TIME_MSEC = 2500;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HitHopper(GearHandler gearHandler, Shooter shooter, TrajectoryDriveController trajController) {
		// these are the subsystems that this auton routine needs to control
		_gearHandler = gearHandler;
		_shooter = shooter;
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
		
		_autonState = AUTON_STATE.MOVE_TO_BOILER_HELLA_FAST;
		_trajController.configureIsHighGear(true);
		_trajController.loadProfile(MoveToHopperTrajectory.LeftPoints, MoveToHopperTrajectory.RightPoints, 1.0, 1.0, MoveToHopperTrajectory.kNumPoints);
		_trajController.enable();
		
		DriverStation.reportWarning("===== Entering Hit Hopper Auton =====", false);
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
      	
		switch(_autonState) {
			case MOVE_TO_BOILER_HELLA_FAST:
				if(_trajController.onTarget()) {
					_trajController.disable();
					_waitStartedTimeStamp = System.currentTimeMillis();
					_autonState = AUTON_STATE.WAIT;
				}
				break;
				
			case WAIT:
				if((System.currentTimeMillis() - _waitStartedTimeStamp) > WAIT_TIME_MSEC) {
					_trajController.loadProfile(MOTION_PROFILE.HOPPER_TO_SHOOTING_POSITION, false);
					_trajController.enable();
					_autonState = AUTON_STATE.MOVE_TO_SHOOTING_POSITION;
				}
				break;
				
			case MOVE_TO_SHOOTING_POSITION:
				if (_trajController.onTarget()) {
					_trajController.disable();
					_autonState = AUTON_STATE.SHOOT;
				}
				break;
				
			case SHOOT:
				break;
				
			case UNDEFINED:
				break;
		}
		
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportWarning("===== Completed CrossBaseLine Auton =====", false);
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