package org.usfirst.frc.team4028.robot.controllers;
import org.usfirst.frc.team4028.robot.util.BeefyMath;
import org.usfirst.frc.team4028.robot.util.CenterGearTrajectory;
import org.usfirst.frc.team4028.robot.util.MoveToBoilerTrajectory;
import org.usfirst.frc.team4028.robot.util.SideGearTrajectory;
import org.usfirst.frc.team4028.robot.util.TrajectoryFollower;
import org.usfirst.frc.team4028.robot.util.TurnAndShootTrajectory;
import org.usfirst.frc.team4028.robot.util.TwoGearLong;
import org.usfirst.frc.team4028.robot.util.TwoGearShort;
import org.usfirst.frc.team4028.robot.util.TwoGearSuperShort;

import java.util.TimerTask;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.DriverStation;

public class TrajectoryDriveController {
	
	private Chassis _chassis;
	private NavXGyro _navX;
	private UpdaterTask _updaterTask;
	private TrajectoryFollower _leftFollower = new TrajectoryFollower("left");
	private TrajectoryFollower _rightFollower = new TrajectoryFollower("right");
	private java.util.Timer _updaterTimer;
	private double _angleDiff;
	private double _direction;
	private double _heading;
	private double _kTurnGyro = -0.01;  // Should be a constant
	private double _kTurnVision = 0.0;
	private double[][] _leftMotionProfile;
	private double[][] _rightMotionProfile;
	private boolean _isEnabled;
	private boolean _isUpdaterTaskRunning;
	private boolean _isVisionTrackingEnabled;
	private int _currentSegment;
	private int _trajectoryNumPoints;
	
	public TrajectoryDriveController(Chassis chassis, NavXGyro navX) {
		_chassis = chassis;
		_navX = navX;
		_leftFollower.configure(0.25,  0.0,  0.0,  0.31,  0.0);
		_rightFollower.configure(0.25,  0.0,  0.0,  0.31,  0.0);
		_updaterTimer = new java.util.Timer();
		_updaterTask = new UpdaterTask();
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
				break;
				
			case CENTER_GEAR:
				_leftMotionProfile = CenterGearTrajectory.LeftPoints;
				_rightMotionProfile = CenterGearTrajectory.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = CenterGearTrajectory.kNumPoints;
				break;
				
			case MOVE_TO_BOILER:
				if (isBlueAlliance) {
					_leftMotionProfile = MoveToBoilerTrajectory.RightPoints;
					_rightMotionProfile = MoveToBoilerTrajectory.LeftPoints;
					_heading = 1.0;
				} else {
					_leftMotionProfile = MoveToBoilerTrajectory.LeftPoints;
					_rightMotionProfile = MoveToBoilerTrajectory.RightPoints;
					_heading = -1.0;
				}
				_direction = -1.0;
				_trajectoryNumPoints = MoveToBoilerTrajectory.kNumPoints;
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
				break;
				
			case TURN_AND_SHOOT:
				_leftMotionProfile = TurnAndShootTrajectory.LeftPoints;
				_rightMotionProfile = TurnAndShootTrajectory.RightPoints;
				_direction = -1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TurnAndShootTrajectory.kNumPoints;
				break;
				
			case TWO_GEAR_LONG:
				_leftMotionProfile = TwoGearLong.LeftPoints;
				_rightMotionProfile = TwoGearLong.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearLong.kNumPoints;
				break;
				
			case TWO_GEAR_SHORT_FWD:
				_leftMotionProfile = TwoGearShort.LeftPoints;
				_rightMotionProfile = TwoGearShort.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearShort.kNumPoints;
				break;
				
			case TWO_GEAR_SHORT_REV:
				_leftMotionProfile = TwoGearShort.LeftPoints;
				_rightMotionProfile = TwoGearShort.RightPoints;
				_direction = -1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearShort.kNumPoints;
				break;
				
			case TWO_GEAR_SUPER_SHORT:
				_leftMotionProfile = TwoGearSuperShort.LeftPoints;
				_rightMotionProfile = TwoGearSuperShort.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearSuperShort.kNumPoints;
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
			
			double leftPower = _direction * _leftFollower.calculate(distanceL, _leftMotionProfile, currentSegment);
			double rightPower = _direction * _rightFollower.calculate(distanceR, _rightMotionProfile, currentSegment);
			
			double goalHeading = _leftFollower.getHeading();
			double goalHeadingInDegrees = _heading * BeefyMath.arctan(goalHeading);
			double observedHeading = _navX.getYaw();

			double turn = _kTurnGyro * (observedHeading - goalHeadingInDegrees);
			
			_chassis.TankDrive(leftPower - turn, rightPower + turn);
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
		return _leftFollower.getCurrentSegment();
	}
	
	public void startTrajectoryController() {
		_isUpdaterTaskRunning = true;
		_updaterTimer.scheduleAtFixedRate(_updaterTask, 0, 20);
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