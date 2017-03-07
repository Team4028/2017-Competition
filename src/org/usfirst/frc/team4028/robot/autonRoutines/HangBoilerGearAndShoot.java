package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.HangGearController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Hang the Gear on the Boiler Side Peg" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	6.Mar.2017	Initial Version
//------------------------------------------------------
//=====> For Changes see Sebas
public class HangBoilerGearAndShoot {
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private Chassis _chassis;
	private NavXGyro _navX;
	private Shooter _shooter;
	private TrajectoryDriveController _trajController;
	private HangGearController _hangGearController;
	
	private enum AUTON_STATE {
		UNDEFINED,
		MOVE_TO_TARGET,
		RUN_GEAR_SEQUENCE,
		MOVE_TO_BOILER,
		SHOOT
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	// define class level constants
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HangBoilerGearAndShoot(GearHandler gearHandler, Chassis chassis, NavXGyro navX, HangGearController hangGear, Shooter shooter) {
		// these are the subsystems that this auton routine needs to control
		_gearHandler = gearHandler;
		_chassis = chassis;
		_navX = navX;
		_hangGearController = hangGear;
		_shooter = shooter;
		_trajController = new TrajectoryDriveController(_chassis, _navX, false);
		_trajController.startTrajectoryController();
		DriverStation.reportError("Auton Initialized", false);
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	// execute any logic to initialize this object before ExecuteRentrant is called
	public void Initialize() {
		_autonStartedTimeStamp = System.currentTimeMillis();
		_isStillRunning = true;
		_autonState = AUTON_STATE.MOVE_TO_TARGET;
		
		_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
		_trajController.loadProfile(MOTION_PROFILE.BOILER_GEAR, false);
		_trajController.enable();
		DriverStation.reportError(Double.toString(_trajController.getCurrentHeading()), false);
		DriverStation.reportWarning("===== Entering HangBoilerSideGear Auton =====", false);
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
    	} else {
    		DriverStation.reportError("Zeroed", false);
    		_gearHandler.MoveGearToScorePosition();
    	}
      	
      	switch (_autonState) {
      		case MOVE_TO_TARGET:
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				DriverStation.reportError(Double.toString(_trajController.getCurrentHeading()), false);
      				_hangGearController.Initialize();
      				_autonState = AUTON_STATE.RUN_GEAR_SEQUENCE;
      			}
      			break;
      			
      		case RUN_GEAR_SEQUENCE:
      			boolean isStillRunning = _hangGearController.ExecuteRentrant();
      			if (!isStillRunning) {
      				_trajController.loadProfile(MOTION_PROFILE.MOVE_TO_BOILER, false);
      				_trajController.enable();
      				_autonState = AUTON_STATE.MOVE_TO_BOILER;
      			}
      			break;
      			
      		case MOVE_TO_BOILER:
      			if(_trajController.onTarget()) {
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
			DriverStation.reportWarning("===== Completed HangBoilerGear Auton =====", false);
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