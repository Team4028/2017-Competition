package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ALLIANCE_COLOR;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.AutoShootController;
import org.usfirst.frc.team4028.robot.controllers.ChassisAutoAimController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

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
	private AutoShootController _autoShootController;
	private ChassisAutoAimController _autoAim;
	private GearHandler _gearHandler;
	private Shooter _shooter;
	private TrajectoryDriveController _trajController;
	private ALLIANCE_COLOR _allianceColor;
	
	private int _targetShootingDistanceInInches;
	private static final int RED_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES = 114;
	private static final int BLUE_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES = 156;
	
	private enum AUTON_STATE {
		UNDEFINED,
		MOVE_TO_BOILER_HELLA_FAST_X,
		WAIT,
		MOVE_TO_SHOOTING_POSITION,
		VISION_TURN,
		SHOOT
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private long _waitStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	// define class level constants
	private static final int WAIT_TIME_MSEC = 1300;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HitHopper(AutoShootController autoShoot, ChassisAutoAimController autoAim, GearHandler gearHandler, Shooter shooter, TrajectoryDriveController trajController, ALLIANCE_COLOR allianceColor) {
		// these are the subsystems that this auton routine needs to control
		_autoShootController = autoShoot;
		_autoAim = autoAim;
		_gearHandler = gearHandler;
		_shooter = shooter;
		_trajController = trajController;
		_allianceColor = allianceColor;
		
		switch(_allianceColor) {
		case BLUE_ALLIANCE:
			_targetShootingDistanceInInches = BLUE_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES;
			break;
			
		case RED_ALLIANCE:
			_targetShootingDistanceInInches = RED_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES;
			break;
		}
		DriverStation.reportError("Auton Initialized", false);
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	// execute any logic to initialize this object before ExecuteRentrant is called
	public void Initialize() {
		_autonStartedTimeStamp = System.currentTimeMillis();
		_isStillRunning = true;
		_autonState = AUTON_STATE.MOVE_TO_BOILER_HELLA_FAST_X;
		_autoShootController.LoadTargetDistanceInInches(_targetShootingDistanceInInches);
		_autoShootController.StopShooter();
		
		_trajController.configureIsHighGear(true);
		switch(_allianceColor) {
			case BLUE_ALLIANCE:
				_trajController.loadProfile(MOTION_PROFILE.MOVE_TO_HOPPER_BLUE_X, true);
				break;
				
			case RED_ALLIANCE:
				_trajController.loadProfile(MOTION_PROFILE.MOVE_TO_HOPPER_RED_X, false);
				break;
		}
		_trajController.enable();
		
		_autoShootController.EnableBoilerCam();
		
		DriverStation.reportWarning("===== Entering Hit Hopper Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() {
		switch(_autonState) {
			case MOVE_TO_BOILER_HELLA_FAST_X:
				if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      	      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      	      		//			we must treat it as a Reentrant function
      	      		//			and automatically recall it until complete
      	    		_gearHandler.ZeroGearTiltAxisReentrant();
      	    	} else {
      	    		DriverStation.reportError("Gear Tilt Zero completed!", false);
      	    		_gearHandler.MoveGearToScorePosition();
      	    	}
				
				if(_trajController.onTarget()) {
					_trajController.disable();
					_waitStartedTimeStamp = System.currentTimeMillis();
					_autonState = AUTON_STATE.WAIT;
				}
				break;
				
			case WAIT:
				if((System.currentTimeMillis() - _waitStartedTimeStamp) > 800) {
					_autoShootController.RunShooterAtTargetSpeed();
				}
				
				if((System.currentTimeMillis() - _waitStartedTimeStamp) > WAIT_TIME_MSEC) {
					switch(_allianceColor) {
						case BLUE_ALLIANCE:
							_trajController.loadProfile(MOTION_PROFILE.TWO_GEAR_SHORT_FWD, true);
							break;
							
						case RED_ALLIANCE:
							_trajController.loadProfile(MOTION_PROFILE.TWO_GEAR_SHORT_FWD, false);
							break;
					}
					_trajController.enable();
					_autonState = AUTON_STATE.MOVE_TO_SHOOTING_POSITION;
				}
				break;
				
			case MOVE_TO_SHOOTING_POSITION:
				_autoShootController.RunShooterAtTargetSpeed();
				if (_trajController.onTarget()) {
					_trajController.disable();
					
					_autonState = AUTON_STATE.VISION_TURN;
				}
				break;
				
			case VISION_TURN:
				_autoShootController.AimWithVision();
      			
      			if(_autoShootController.IsReadyToShoot()) {
      				// start shooter feeder motors
      				_shooter.ToggleRunShooterFeeder();
      				
      				// chg state
      				_autonState = AUTON_STATE.SHOOT;
      				DriverStation.reportError("PEW PEW PEW PEW PEW", false);
      			}
				break;
				
			case SHOOT:
				_shooter.RunShooterFeederReentrant();
				
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