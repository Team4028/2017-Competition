package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.AutoShootController;
import org.usfirst.frc.team4028.robot.controllers.ChassisAutoAimController;
import org.usfirst.frc.team4028.robot.controllers.HangGearController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Hang the Gear on the Center Peg And Shoot" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	6.Mar.2017	Initial Version
//------------------------------------------------------
//=====> For Changes see Sebas
public class HangCenterGearAndShoot {
	// define class level variables for Robot subsystems
	private AutoShootController _autoShootController;
	private GearHandler _gearHandler;
	private Chassis _chassis;
	private NavXGyro _navX;
	private ChassisAutoAimController _autoAim;
	private Shooter _shooter;
	private TrajectoryDriveController _trajController;
	private HangGearController _hangGearController;
	
	private enum AUTON_STATE {
		UNDEFINED, 
		MOVE_TO_TARGET,
		RUN_GEAR_SEQUENCE,
		MOVE_BACK,
		TURN,
		SHOOT,
		FINISHED
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	// define class level constants
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HangCenterGearAndShoot(AutoShootController autoShoot, GearHandler gearHandler, Chassis chassis, ChassisAutoAimController autoAim, NavXGyro navX, 
			HangGearController hangGear, Shooter shooter, TrajectoryDriveController trajController) {
		// these are the subsystems that this auton routine needs to control
		_autoShootController = autoShoot;
		_gearHandler = gearHandler;
		_chassis = chassis;
		_navX = navX;
		_hangGearController = hangGear;
		_shooter = shooter;
		_autoAim = autoAim;
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
		_autonState = AUTON_STATE.MOVE_TO_TARGET;
		
		_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
		_trajController.configureIsHighGear(false);
		_trajController.loadProfile(MOTION_PROFILE.CENTER_GEAR, false);
		_trajController.enable();
		
		// chg vision camera to Boiler
		_autoShootController.EnableGearCam();
		
		DriverStation.reportError(Double.toString(_trajController.getCurrentHeading()), false);
		DriverStation.reportWarning("===== Entering HangCenterGear Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() { 	
      	switch (_autonState) {
      	
      		case MOVE_TO_TARGET:
      			if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      	      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      	      		//			we must treat it as a Reentrant function
      	      		//			and automatically recall it until complete
      	    		_gearHandler.ZeroGearTiltAxisReentrant();
      	    	} else {
      	    		DriverStation.reportError("Gear Tilt Zero completed!", false);
      	    		_gearHandler.MoveGearToScorePosition();
      	    	}
      			
      			// if we are on the 1st step of the Motion Profile
      			if (_trajController.getCurrentSegment() == 1) {
      				_trajController.isVisionTrackingEnabled(true);
      			}
      			// if the motion profile is complete
      			else if (_trajController.onTarget()) {
      				_trajController.disable();
      				_trajController.isVisionTrackingEnabled(false);
      				
      				DriverStation.reportError(Double.toString(_trajController.getCurrentHeading()), false);
      				_hangGearController.Initialize();
      				
      				// chg state
      				_autonState = AUTON_STATE.RUN_GEAR_SEQUENCE;
      				DriverStation.reportError("===> Chg state from MOVE_TO_TARGET to RUN_GEAR_SEQUENCE", false);
      			}
      			break;
      			
      		case RUN_GEAR_SEQUENCE:
      			boolean isStillRunning = _hangGearController.ExecuteRentrant();
      			
      			if (!isStillRunning) {
      				// chg vision camera to Boiler
      				_autoShootController.EnableBoilerCam();
      				
      				// load new motion profile
      				_trajController.loadProfile(MOTION_PROFILE.TWO_GEAR_SHORT_REV, false);
      				
      				// start thread
      				_trajController.enable();
      				
      				// chg state
      				_autonState = AUTON_STATE.MOVE_BACK;
      				DriverStation.reportError("===> Chg state from RUN_GEAR_SEQUENCE to MOVE_BACK", false);
      			}
      			break;
      			
      		case MOVE_BACK:
      			if (_trajController.onTarget()) {
      				// disable the thread
      				_trajController.disable();
      				
      				// set target delta turn angle
      				_autoAim.loadNewTarget(-70.0);
      				
      				// chg state
      				_autonState = AUTON_STATE.TURN;
      				DriverStation.reportError("===> Chg state from MOVE_BACK to TURN", false);
      			}
      			break;
      			
      		case TURN:
      			// call turn controller
      			_autoAim.update();
      			
      			// have we reached the target angle w/i the threshhold ?
      			if (_autoAim.onTarget()) {
      				
      				// chg state
      				_autonState = AUTON_STATE.SHOOT;
      				DriverStation.reportError("===> Chg state from TURN to SHOOT", false);
      			}
      			break;
      			
      		case SHOOT:
      			_autoShootController.AimAndShootWhenReady();
      			
      			// TODO: need to decide how to exit
      			//_autonState = AUTON_STATE.FINISHED;
      			break;
      			
      		case FINISHED:
      			_isStillRunning = false;
      			break;
      			
      		case UNDEFINED:
      			break;
      	}
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportWarning("===== Completed HangCenterGear Auton =====", false);
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