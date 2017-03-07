package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.AUTON_MODE;
import org.usfirst.frc.team4028.robot.controllers.ChassisAutoAimController;
import org.usfirst.frc.team4028.robot.controllers.HangGearController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.util.TwoGearLong;
import org.usfirst.frc.team4028.robot.util.TwoGearShort;
import org.usfirst.frc.team4028.robot.util.TwoGearSuperShort;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for hanging two gears in auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	25.Feb.2017	Initial Version
//1.0 		Sebas		4.Mar.2017	Added Motion Profile + Hang Gear 1
//------------------------------------------------------
//=====> For Changes see Sebas
public class TwoGear {
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private Chassis _chassis;
	private NavXGyro _navX;
	private TrajectoryDriveController _trajController;
	private ChassisAutoAimController _autoAim;
	private HangGearController _hangGearController;
	
	private enum AUTON_STATE {
		UNDEFINED, 
		MOVE_TO_TARGET_1,
		RUN_GEAR_SEQUENCE_1, 
		TURN_RIGHT_1,
		MOVE_ACROSS,
		TURN_RIGHT_2,
		MOVE_TO_GEAR,
		MOVE_FROM_GEAR,
		TURN_RIGHT_3,
		MOVE_ACROSS_2,
		TURN_RIGHT_4,
		MOVE_TO_TARGET_2,
		RUN_GEAR_SEQUENCE_2
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	// define class level constants
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public TwoGear(GearHandler gearHandler, Chassis chassis, NavXGyro navX, HangGearController hangGear) {
		// these are the subsystems that this auton routine needs to control
		_gearHandler = gearHandler;
		_chassis = chassis;
		_navX = navX;
		_hangGearController = hangGear;
		_autoAim = new ChassisAutoAimController(_chassis, _navX);
		_trajController = new TrajectoryDriveController(_chassis, _navX);
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
		_autonState = AUTON_STATE.MOVE_TO_TARGET_1;
		
		_autoAim.loadNewTarget(90.0);
		_trajController.loadProfile(AUTON_MODE.HANG_CENTER_GEAR, false);
		_trajController.enable();
		DriverStation.reportError(Double.toString(_trajController.getCurrentHeading()), false);
		DriverStation.reportWarning("===== Entering HangCenterGear Auton =====", false);
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
      	
      	switch (_autonState) {
      		case MOVE_TO_TARGET_1:
      			if (_gearHandler.hasTiltAxisBeenZeroed()) {
      				_gearHandler.MoveGearToScorePosition();
      			}
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				DriverStation.reportError(Double.toString(_trajController.getCurrentHeading()), false);
      				_hangGearController.Initialize();
      				_autonState = AUTON_STATE.RUN_GEAR_SEQUENCE_1;
      			}
      			break;
      			
      		case RUN_GEAR_SEQUENCE_1:
      			boolean isStillRunning = _hangGearController.ExecuteRentrant();
      			if (!isStillRunning) {
      				_autonState = AUTON_STATE.TURN_RIGHT_1;
      			}
      			break;
      			
      		case TURN_RIGHT_1:
      			_autoAim.update();
      			_trajController.loadProfile(TwoGearLong.LeftPoints, TwoGearLong.RightPoints, 1.0, 1.0, TwoGearLong.kNumPoints);
      			if(_autoAim.onTarget()) {
      				_trajController.enable();
      				_autonState = AUTON_STATE.MOVE_ACROSS;
      			}
      			break;
      			
      		case MOVE_ACROSS:
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				_autonState = AUTON_STATE.TURN_RIGHT_2;
      			}
      			break;
      			
      		case TURN_RIGHT_2:
      			_autoAim.update();
      			_gearHandler.MoveGearToFloorPositionReentrant();
      			_trajController.loadProfile(TwoGearShort.LeftPoints, TwoGearShort.RightPoints, 1.0, 1.0, TwoGearShort.kNumPoints);
      			if(_autoAim.onTarget()) {
      				_trajController.enable();
      				_autonState = AUTON_STATE.MOVE_TO_GEAR;
      			}
      			break;
      			
      		case MOVE_TO_GEAR:
      			_gearHandler.SpinInfeedWheelsVBus(1.0);
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				_trajController.loadProfile(TwoGearShort.LeftPoints, TwoGearShort.RightPoints, -1.0, 1.0, TwoGearShort.kNumPoints);
      				_trajController.enable();
      				_autonState = AUTON_STATE.MOVE_FROM_GEAR;
      			}
      			break;
      			
      		case MOVE_FROM_GEAR:
      			_gearHandler.SpinInfeedWheelsVBus(1.0);
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				_autonState = AUTON_STATE.TURN_RIGHT_3;
      			}
      			break;
      			
      		case TURN_RIGHT_3:
      			_gearHandler.SpinInfeedWheelsVBus(0.0);
      			_autoAim.update();
      			_gearHandler.MoveGearToScorePosition();
      			_trajController.loadProfile(TwoGearLong.LeftPoints, TwoGearLong.LeftPoints, 1.0, 1.0, TwoGearLong.kNumPoints);
      			if(_autoAim.onTarget()) {
      				_trajController.enable();
      				_autonState = AUTON_STATE.MOVE_ACROSS_2;
      			}
      			break;
      			
      		case MOVE_ACROSS_2:
      			if(_trajController.onTarget()) {
      				_trajController.disable();
      				_autonState = AUTON_STATE.TURN_RIGHT_4;
      			}
      			break;
      			
      		case TURN_RIGHT_4:
      			_autoAim.update();
      			_trajController.loadProfile(TwoGearSuperShort.LeftPoints, TwoGearSuperShort.LeftPoints, 1.0, 1.0, TwoGearSuperShort.kNumPoints);
      			if(_autoAim.onTarget()) {
      				_trajController.enable();
      				_autonState = AUTON_STATE.MOVE_TO_TARGET_2;
      			}
      			break;
      			
      		case MOVE_TO_TARGET_2:
      			if(_trajController.onTarget()) {
      				_trajController.disable();
      				_hangGearController.Initialize();
      				_autonState = AUTON_STATE.RUN_GEAR_SEQUENCE_2;
      			}
      			
      			
      		case RUN_GEAR_SEQUENCE_2:
      			boolean isStillRunning2 = _hangGearController.ExecuteRentrant();
      			if (!isStillRunning2) {
      				double totalTime = System.currentTimeMillis() - _autonStartedTimeStamp;
      				DriverStation.reportError(Double.toString(totalTime), false);
      			}
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
