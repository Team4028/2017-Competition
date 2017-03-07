package org.usfirst.frc.team4028.robot.controllers;
import org.usfirst.frc.team4028.robot.util.BeefyMath;
import org.usfirst.frc.team4028.robot.util.CenterGearTrajectory;
import org.usfirst.frc.team4028.robot.util.SideGearTrajectory;
import org.usfirst.frc.team4028.robot.util.Trajectory;
import org.usfirst.frc.team4028.robot.util.TrajectoryFollower;

import java.util.TimerTask;

import org.usfirst.frc.team4028.robot.Robot;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.AUTON_MODE;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.DriverStation;

public class TrajectoryDriveController extends Robot {
	
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
	
	public void loadProfile(AUTON_MODE autonMode, boolean isBlueAlliance) {
		reset();
		_angleDiff = 0.0;
		switch(autonMode) {
			case CROSS_BASE_LINE:
				break;
				
			case HANG_BOILER_GEAR:
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
				
			case HANG_CENTER_GEAR: 
				_leftMotionProfile = CenterGearTrajectory.LeftPoints;
				_rightMotionProfile = CenterGearTrajectory.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = CenterGearTrajectory.kNumPoints;
				break;
				
			case HANG_RETRIEVAL_GEAR: 
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
		}
	}
	
	public void reset() {
		_leftFollower.reset();
		_rightFollower.reset();
	}
	
	public int getFollowerCurrentSegment() {
		return _leftFollower.getCurrentSegment();
	}
	
	public int getNumSegments() {
		return _leftFollower.getNumSegments();
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
	
	public int getCurrentSegment() {
		return _leftFollower.getCurrentSegment();
	}
	
	public double getCurrentHeading() {
		return _navX.getYaw();
	}
	
	public double getAngleDiff() {
		return _angleDiff;
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