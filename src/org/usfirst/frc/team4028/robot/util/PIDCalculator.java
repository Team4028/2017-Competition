package org.usfirst.frc.team4028.robot.util;

import edu.wpi.first.wpilibj.util.BoundaryException;

public class PIDCalculator {
	
	private double _p; // "proportional" term
	private double _i; // "integral" term
	private double _d; // "derivative" term
	private double _maximumOutput = 0.7; 
	private double _minimumOutput = -0.7;
	private double _prevError = 0.0;
	private double _totalError = 0.0;
	private double _totalErrorCeiling = 15.0;
	private double _setpoint = 0.0;
	private double _error = 0.0;
	private double _result = 0.0;
	private double _deadband = 0.5;
	
	public PIDCalculator(double Kp, double Ki, double Kd) {
		_p = Kp;
		_i = Ki;
		_d = Kd;
	}
	
	public double calculate (double input) {
		_error = _setpoint - input;		
		if (Math.abs(_error) < 5.0) {
			if (Math.abs(_totalError + _error) < _totalErrorCeiling) {
				_totalError += _error;
			} else {
			}
		} else {
			_totalError = 0.0;
		}
		
		double proportionalError = Math.abs(_error) < _deadband ? 0 : _error;
		
		_result = (_p * proportionalError + _i * _totalError + _d * (_error - _prevError));
		
		if (_result > _maximumOutput) {
            _result = _maximumOutput;
        } else if (_result < _minimumOutput) {
            _result = _minimumOutput;
        }
        return _result;  
	}
	
	public void setDeadband (double deadband) {
		_deadband = deadband;
	}
	
	public void setOutputRange(double minimumOutput, double maximumOutput) 
	{
		if (minimumOutput > maximumOutput) {
			throw new BoundaryException("Lower bound is greater than upper bound");
		}
		
		_minimumOutput = minimumOutput;
		_maximumOutput = maximumOutput;
	}
	
	public void setSetpoint(double setpoint) {
        _setpoint = setpoint;
    }
	
	public boolean onTarget() {
        return (Math.abs(_error) < _deadband);
    }

    /**
     * Reset all internal terms.
     */
    public void reset() {
        _prevError = 0;
        _totalError = 0;
        _result = 0;
        _setpoint = 0;
    }
    
    public void resetTotalError() {
    	_totalError = 0;
    }
	
	public double getError() {
        return _error;
    }
	
	public double getSetpoint() {
		return _setpoint;
	}
}
