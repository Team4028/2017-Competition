package org.usfirst.frc.team4028.robot.controllers;
import org.usfirst.frc.team4028.robot.util.BeefyMath;
import org.usfirst.frc.team4028.robot.util.CenterGearTrajectory;
import org.usfirst.frc.team4028.robot.util.HopperToBoilerTrajectory;
import org.usfirst.frc.team4028.robot.util.MoveToBoilerTrajectory;
import org.usfirst.frc.team4028.robot.util.MoveToHopperTrajectory;
import org.usfirst.frc.team4028.robot.util.SideGearTrajectory;
import org.usfirst.frc.team4028.robot.util.TrajectoryFollower;
import org.usfirst.frc.team4028.robot.util.TurnAndShootTrajectory;
import org.usfirst.frc.team4028.robot.util.TwoGearLong;
import org.usfirst.frc.team4028.robot.util.TwoGearShort;
import org.usfirst.frc.team4028.robot.util.TwoGearSuperShort;

import java.util.TimerTask;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.sensors.RoboRealmClient;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryDriveController {
	
	private Chassis _chassis;
	private NavXGyro _navX;
	private UpdaterTask _updaterTask;
	private TrajectoryFollower _leftFollower = new TrajectoryFollower();
	private TrajectoryFollower _rightFollower = new TrajectoryFollower();
	private RoboRealmClient _roboRealm;
	private java.util.Timer _updaterTimer;
	private double _angleDiff;
	private double _currentVisionError;
	private double _direction;
	private double _heading;
	private double _kTurnGyro;
	private double _kTurnVision = -0.006;
	private double _leftPower;
	private double _rightPower;
	private double _setVisionError;
	private double _turn;
	private double _visionTurnThreshold = 0.15;
	private double[][] _leftMotionProfile;
	private double[][] _rightMotionProfile;
	private boolean _isAutoStopEnabled = false;
	private boolean _isEnabled;
	private boolean _isUpdaterTaskRunning;
	private boolean _isVisionTrackingEnabled;
	private int _currentSegment;
	private int _trajectoryNumPoints;
	
	public TrajectoryDriveController(Chassis chassis, NavXGyro navX, RoboRealmClient roboRealm) {
		_chassis = chassis;
		_navX = navX;
		_updaterTimer = new java.util.Timer();
		_updaterTask = new UpdaterTask();
		_roboRealm = roboRealm;
		startTrajectoryController();
	}
	
	public void configureIsHighGear(boolean isHighGear) {
		if(isHighGear) {
			_leftFollower.configure(0.18, 0.0, 0.0, 0.15, 0.0);
			_rightFollower.configure(0.18, 0.0, 0.0, 0.15, 0.0);
			_kTurnGyro = -0.01;
		} else {
			_leftFollower.configure(0.27,  0.0,  0.0,  0.29,  0.0);
			_rightFollower.configure(0.27,  0.0,  0.0,  0.29,  0.0);
			_kTurnGyro = -0.01;
		}
	}
	
	public boolean onTarget() {
		if(_currentSegment == (_trajectoryNumPoints - 1)) {
			return true;
		} else {
			return false;
		}
	}
	
	public void loadProfile(double[][] leftProfile, double[][] rightProfile, double direction, double heading, int numPoints) {
		reset();
		_leftMotionProfile = leftProfile;
		_rightMotionProfile = rightProfile;
		_direction = direction;
		_heading = heading;
		_trajectoryNumPoints = numPoints;
		_angleDiff = 0.0;
	}
	
	public void loadProfile(MOTION_PROFILE motionProfile, boolean isBlueAlliance) {
		reset();
		_angleDiff = 0.0;
		switch(motionProfile) {
			case BOILER_GEAR:
				if (isBlueAlliance) {
					_leftMotionProfile = SideGearTrajectory.LeftPoints;
					_rightMotionProfile = SideGearTrajectory.RightPoints;
					_heading = 1.0;
				} else {
					_leftMotionProfile = SideGearTrajectory.RightPoints;
					_rightMotionProfile = SideGearTrajectory.LeftPoints;
					_heading = -1.0;
				}
				_direction = 1.0;
				_trajectoryNumPoints = SideGearTrajectory.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case CENTER_GEAR:
				_leftMotionProfile = CenterGearTrajectory.LeftPoints;
				_rightMotionProfile = CenterGearTrajectory.LeftPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = CenterGearTrajectory.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case HOPPER_TO_SHOOTING_POSITION:
				_leftMotionProfile = HopperToBoilerTrajectory.LeftPoints;
				_rightMotionProfile = HopperToBoilerTrajectory.RightPoints;
				_direction = -1.0;
				_heading = 1.0;
				_trajectoryNumPoints = HopperToBoilerTrajectory.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);
				break;
				
			case MOVE_TO_BOILER:
				if (isBlueAlliance) {
					_leftMotionProfile = MoveToBoilerTrajectory.LeftPoints;
					_rightMotionProfile = MoveToBoilerTrajectory.RightPoints;
					_heading = -1.0;
				} else {
					_leftMotionProfile = MoveToBoilerTrajectory.RightPoints;
					_rightMotionProfile = MoveToBoilerTrajectory.LeftPoints;
					_heading = 1.0;
				}
				_direction = -1.0;
				_trajectoryNumPoints = MoveToBoilerTrajectory.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case MOVE_TO_HOPPER:
				_leftMotionProfile = MoveToHopperTrajectory.LeftPoints;
				_leftMotionProfile = MoveToHopperTrajectory.RightPoints;
				_heading = 1.0;
				_direction = -1.0;
				_trajectoryNumPoints = MoveToHopperTrajectory.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);
				break;
				
			case RETRIEVAL_GEAR:
				if (isBlueAlliance) {
					_leftMotionProfile = SideGearTrajectory.RightPoints;
					_rightMotionProfile = SideGearTrajectory.LeftPoints;
					_heading = -1.0;
				} else {
					_leftMotionProfile = SideGearTrajectory.LeftPoints;
					_rightMotionProfile = SideGearTrajectory.RightPoints;
					_heading = 1.0;
				}
				_direction = 1.0;
				_trajectoryNumPoints = SideGearTrajectory.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case TURN_AND_SHOOT:
				_leftMotionProfile = TurnAndShootTrajectory.LeftPoints;
				_rightMotionProfile = TurnAndShootTrajectory.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TurnAndShootTrajectory.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case TWO_GEAR_LONG:
				_leftMotionProfile = TwoGearLong.LeftPoints;
				_rightMotionProfile = TwoGearLong.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearLong.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case TWO_GEAR_SHORT_FWD:
				_leftMotionProfile = TwoGearShort.LeftPoints;
				_rightMotionProfile = TwoGearShort.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearShort.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case TWO_GEAR_SHORT_REV:
				_leftMotionProfile = TwoGearShort.LeftPoints;
				_rightMotionProfile = TwoGearShort.RightPoints;
				_direction = -1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearShort.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case TWO_GEAR_SUPER_SHORT:
				_leftMotionProfile = TwoGearSuperShort.LeftPoints;
				_rightMotionProfile = TwoGearSuperShort.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearSuperShort.kNumPoints;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
		}
	}
	
	public void reset() {
		_leftFollower.reset();
		_rightFollower.reset();
	}
	
	public void update(int currentSegment) {
		if (!_isEnabled) {
			DriverStation.reportError("Not Enabled", false);
			_chassis.TankDrive(0, 0);
		}
		
		if (onTarget()) {
			DriverStation.reportError("At Target", false);
			_chassis.TankDrive(0, 0);
		} else {
			double distanceL = _direction * _chassis.getLeftEncoderCurrentPosition();
			double distanceR = _direction * _chassis.getRightEncoderCurrentPosition();
			

			if(_isVisionTrackingEnabled && _roboRealm.get_isVisionDataValid()) {
				//setIsFeedbackDisabled(true);
				_currentVisionError = _roboRealm.get_Angle();
		
				if(_setVisionError != _currentVisionError) {
					_setVisionError = _currentVisionError;
				}
				_turn = _kTurnVision * _setVisionError;
				if (_turn > _visionTurnThreshold) {
					_turn = _visionTurnThreshold;
				} else if (_turn < (-1.0 * _visionTurnThreshold)) {
					_turn = -1.0 * _visionTurnThreshold;
				} else {
				}
			
			} else {
				//setIsFeedbackDisabled(false);
				double goalHeading = _leftFollower.getHeading();
				double goalHeadingInDegrees = _heading * BeefyMath.arctan(goalHeading);
				double observedHeading = _navX.getYaw();

				_turn = _kTurnGyro * (observedHeading - goalHeadingInDegrees);
			}
			
			if(_isAutoStopEnabled && _roboRealm.get_isVisionDataValid()) {
				if (!_roboRealm.get_isInGearHangPosition()) {
					_leftPower = _direction * _leftFollower.calculate(distanceL, _leftMotionProfile, currentSegment);
					_rightPower = _direction * _rightFollower.calculate(distanceR, _rightMotionProfile, currentSegment);
				} else {
					_leftPower = 0.0;
					_rightPower = 0.0;
					DriverStation.reportError("AUTO STOP!", false);
				}
			} else {
				_leftPower = _direction * _leftFollower.calculate(distanceL, _leftMotionProfile, currentSegment);
				_rightPower = _direction * _rightFollower.calculate(distanceR, _rightMotionProfile, currentSegment);
			}
			
			
			_chassis.TankDrive(_leftPower - _turn, _rightPower + _turn);
		}
	}
	
	public void enable() {
		_leftFollower.setTrajectoryNumPoints(_trajectoryNumPoints);
		_rightFollower.setTrajectoryNumPoints(_trajectoryNumPoints);
		_leftFollower.reset();
		_rightFollower.reset();
		_chassis.ZeroDriveEncoders();
		_navX.zeroYaw();
		_isEnabled = true;
		_currentSegment = 0;
		DriverStation.reportError("Enabled", false);
	}
	
	public void disable() {
		_isEnabled = false;
	}
	
	public boolean isEnable() {
		return _isEnabled;
	}
	
	public void isVisionTrackingEnabled(boolean isEnabled) {
		_isVisionTrackingEnabled = isEnabled;
	}
	
	public double getAngleDiff() {
		return _angleDiff;
	}
	
	public int getCurrentSegment() {
		return _leftFollower.getCurrentSegment();
	}
	
	public double getCurrentHeading() {
		return _navX.getYaw();
	}
	
	public int getFollowerCurrentSegment() {
		return _currentSegment;
	}
	
	public void setIsFeedbackDisabled(boolean isDisabled) {
		_leftFollower.setIsFeedbackDisabled(isDisabled);
		_rightFollower.setIsFeedbackDisabled(isDisabled);
	}
	
	public void startTrajectoryController() {
		_isUpdaterTaskRunning = true;
		_updaterTimer.scheduleAtFixedRate(_updaterTask, 0, 20);
	}
	
	public void OutputToSmartDashboard() {
		if(_roboRealm.get_isVisionDataValid()) {
			SmartDashboard.putNumber("Vision Error", _roboRealm.get_Angle());
		}
	}
	
	private class UpdaterTask extends TimerTask {
		public void run() {
			while(_isUpdaterTaskRunning) {
				if (_isEnabled) {
					if (_currentSegment != (_trajectoryNumPoints - 1)) {
						update(_currentSegment);
						_currentSegment = _currentSegment + 1;
					}	
				}
				try {
					Thread.sleep(20);
				}
				catch(InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}