package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.utilities.LogData;
import org.usfirst.frc.team4028.robot.utilities.GeneralUtilities;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the SHOOTER (& Blender) Subsystem
//=====> For Changes see Prat Bruns

//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		Patrick		2/16 8:47		Enabling Blender and Feeder Motors
//	1		Patrick		2/18 5:36		Code Review
//	2		Patrick		2/20 10:02		Code Review on Shooter Testing
//	3		Patrick		2/20 18:47		Updating Values Written to SmartDashboard
//	4		Patrick		2/22 12:34		Making toggle button for Blender/Feeder
//	5		Patrick		3/1	 5:57		Toggle Blender Speed
//	6		Patrick		3/4	 11:06		Updating to Log %Voltage
//	7		Patrick		3/4	 1:31		Changing PID Values
//  8		TomB		9.Mar.2017		Refactor for new infeed 3 motor combo
//-------------------------------------------------------------
public class Shooter 
{
	// =====================================================================
	// 5 DC Motors
	//		1 Talon w/ Encoder, 	PID V Mode		2nd Stage
	//		1 Talon w/ Encoder, 	PID V Mode		1st Stage
	//		1 Talon w/o Encoder,	% VBus Mode		Magic Carpet Motor
	//		1 Talon w/o Encoder,	% VBus Mode		High Speed Infeed Lane
	//		1 Talon w/o Encoder,	% VBus Mode		High Roller
	//
	// 1 Servo
	// 		I Linear Actuator		PWM				Slider
	// =====================================================================
	
	// define class level variables for Robot objects`
	private CANTalon _firstStgMtr;
	private CANTalon _secondStgMtr;
	private CANTalon _magicCarpetMtr;
	private CANTalon _highSpeedInfeedLaneMtr;
	private CANTalon _highRollerMtr;
	
	private Servo _linearActuator;
	
	// define class level working variables
	private double _stg1MtrTargetRPM;
	private double _stg2MtrTargetRPM;
	private boolean _isStg1MtrTargetRPMBumpingUp;
	private boolean _isStg2MtrTargetRPMBumpingUp;
	
	private double _magicCarpetMtrTargetVBus;	
	private double _highSpeedInfeedLaneMtrTargetVBus;
	private double _highRollerMtrTargetVBus;

	private double _currentSliderPosition;
	
	//define class level PID constants
	private static final double FIRST_STAGE_MTG_FF_GAIN = 0.033; //0.0325; //0.034; //0.032; //0.0315; //0.031;
	private static final double FIRST_STAGE_MTG_P_GAIN = 0.325; //0.25; //0.2; //0.1;
	private static final double FIRST_STAGE_MTG_I_GAIN = 0.0;
	private static final double FIRST_STAGE_MTG_D_GAIN = 5.0; //3.0;

	private static final double SECOND_STAGE_MTG_FF_GAIN = 0.03; //0.0274;
	private static final double SECOND_STAGE_MTG_P_GAIN = 0.175; //0.2; //0.15;
	private static final double SECOND_STAGE_MTG_I_GAIN = 0.0;
	private static final double SECOND_STAGE_MTG_D_GAIN = 6.0; //5.0; //4.0;//3.5; //0.0;//5; //6; //0.115;

	//define class level Actuator Constants
	private static final double MAX_THRESHOLD_ACTUATOR = 0.7; 
	private static final double MIN_THRESHOLD_ACTUATOR = 0.4;
	private static final double CHANGE_INTERVAL_ACTUATOR = 0.02;
	private static final double INITIAL_POSITION_ACTUATOR = 0.65;
	
	//define class level Shooter Motor Constants
	private static final double MAX_SHOOTER_RPM = -4400;
	private static final double MIN_SHOOTER_RPM = -3000;
	private static final double SHOOTER_BUMP_RPM = 50;
	private static final double FIRST_STAGE_MTR_DEFAULT_RPM = -3500;
	private static final double SECOND_STAGE_MTR_DEFAULT_RPM = -3200;
	
	// define class level Ball Infeed Motor Constants
	private static final double MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND = -0.7;
	private static final double HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND = -0.7;
	private static final double HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND = 0.7;

	//============================================================================================
	// CONSTRUCTORS FOLLOW
	//============================================================================================
	public Shooter(int firstStgMtrCanBusAddr, 
					int secondStageMtrCanBusAddr, 
					int magicCarpetMtrCanBusAddr, 
					int highSpeedInfeedLaneMtrCanBusAddr, 
					int highRollerMtrCanBusAddr,
					int sliderPWMPort)
	{
		// First Stage Motor
		_firstStgMtr = new CANTalon(firstStgMtrCanBusAddr);
		_firstStgMtr.changeControlMode(CANTalon.TalonControlMode.Speed);	// open loop throttle
		_firstStgMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_firstStgMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	// set encoder to be feedback device
    	_firstStgMtr.reverseSensor(true);  							// do not invert encoder feedback
		_firstStgMtr.enableLimitSwitch(false, false);
        // set the peak and nominal outputs, 12V means full 
		_firstStgMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		_firstStgMtr.configPeakOutputVoltage(0.0f, -12.0f);
    	
		// set closed loop gains in slot0 
		_firstStgMtr.setProfile(0);
		_firstStgMtr.setF(FIRST_STAGE_MTG_FF_GAIN); 
		_firstStgMtr.setP(FIRST_STAGE_MTG_P_GAIN); 
		_firstStgMtr.setI(FIRST_STAGE_MTG_I_GAIN); 
		_firstStgMtr.setD(FIRST_STAGE_MTG_D_GAIN);
				
		// Second Stage Motor
		_secondStgMtr = new CANTalon(secondStageMtrCanBusAddr);
		_secondStgMtr.changeControlMode(CANTalon.TalonControlMode.Speed);	// open loop throttle
		_secondStgMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_secondStgMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	// set encoder to be feedback device
    	_secondStgMtr.reverseSensor(true);  							// do not invert encoder feedback
		_secondStgMtr.enableLimitSwitch(false, false);
    	//_secondStageMtr.reverseOutput(true);
        // set the peak and nominal outputs, 12V means full
		_secondStgMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		_secondStgMtr.configPeakOutputVoltage(0.0f, -12.0f);
		
		// set closed loop gains in slot0
		_secondStgMtr.setProfile(0);
		_secondStgMtr.setF(SECOND_STAGE_MTG_FF_GAIN); 
		_secondStgMtr.setP(SECOND_STAGE_MTG_P_GAIN); 
		_secondStgMtr.setI(SECOND_STAGE_MTG_I_GAIN); 
		_secondStgMtr.setD(SECOND_STAGE_MTG_D_GAIN);
				
		// Magic Carpet Motor
		_magicCarpetMtr = new CANTalon(magicCarpetMtrCanBusAddr);
		_magicCarpetMtr.enableBrakeMode(false);
		_magicCarpetMtr.enableLimitSwitch(false, false);
		
		// High Speed Infeed Lane Motor
		_highSpeedInfeedLaneMtr = new CANTalon(highSpeedInfeedLaneMtrCanBusAddr);
		_highSpeedInfeedLaneMtr.enableBrakeMode(false);
		_highSpeedInfeedLaneMtr.enableLimitSwitch(false, false);
		
		// High Roller Motor
		_highRollerMtr = new CANTalon(highRollerMtrCanBusAddr);
		_highRollerMtr.enableBrakeMode(false);
		_highRollerMtr.enableLimitSwitch(false, false);
		
		// Slider
		_linearActuator = new Servo(sliderPWMPort);
		
		// Default Feeder Subsystem working variables
		_magicCarpetMtrTargetVBus = 0;
		_highSpeedInfeedLaneMtrTargetVBus = 0;
		_highRollerMtrTargetVBus = 0;
		
		// default to bumping rmp up on both shooter motors
		_isStg1MtrTargetRPMBumpingUp = true;
		_isStg2MtrTargetRPMBumpingUp = true;
	}
	
	//============================================================================================
	// METHODS FOLLOW
	//============================================================================================
	
	public void FullStop() 
	{
		FullShooterStop();
		FullShooterFeederStop();
	}
	
	public void FullShooterStop() 
	{
		RunStg1(0);
		RunStg2(0);
		RunHighSpeedInfeedLane(0.0);
		RunMagicCarpet(0.0);
		RunHighRoller(0.0);
	}
	
	public void FullShooterFeederStop() 
	{
		RunMagicCarpet(0.0);
		RunHighRoller(0.0);
	}
	
	//============================================================================================
	// Shooter Motors
	//============================================================================================

	public void RunStg1(double targetRPM)
	{
		_stg1MtrTargetRPM = targetRPM;
		_firstStgMtr.set(_stg1MtrTargetRPM);
		DriverStation.reportWarning("Stage 1 Target RPM = " + targetRPM, false);
		
		ControlHighSpeedLane();
	}
	
	public void RunStg2(double targetRPM)
	{
		_stg2MtrTargetRPM = targetRPM;
		_secondStgMtr.set(_stg2MtrTargetRPM);
		DriverStation.reportWarning("Stage 2 Target RPM = " + targetRPM, false);
		
		ControlHighSpeedLane();
	}
	
	//============================================================================================
	// Set Up Shooter Testing
	//============================================================================================
		
	public void BumpStg1MtrRPMUp()
	{
		// only bump if not already at max
		if(Math.abs(_stg1MtrTargetRPM) <= Math.abs(MAX_SHOOTER_RPM))
		{	
			if(Math.abs(_stg1MtrTargetRPM) >  0)
			{
				// if already turning, just bump
				RunStg1(_stg1MtrTargetRPM -= SHOOTER_BUMP_RPM);
			}
			else
			{
				// if currently stopped, set to default speed
				RunStg1(FIRST_STAGE_MTR_DEFAULT_RPM);
			}
		}
		else
		{
			DriverStation.reportWarning("Stg 1 Mtr Already at MAX ", false);
			_isStg1MtrTargetRPMBumpingUp = false;
		}
	}
	
	public void BumpStg1MtrRPMDown()
	{
		// only bump if not already at min
		if(Math.abs(_stg1MtrTargetRPM) > Math.abs(MIN_SHOOTER_RPM))
		{
			RunStg1(_stg1MtrTargetRPM += SHOOTER_BUMP_RPM);
		}
		else
		{
			DriverStation.reportWarning("Stg 1 Mtr Already at MIN ", false);
			_isStg1MtrTargetRPMBumpingUp = true;
		}
	}
	
	public void BumpStg2MtrRPMUp()
	{
		// only bump if not already at max
		if(Math.abs(_stg2MtrTargetRPM) <= Math.abs(MAX_SHOOTER_RPM))
		{	
			if(Math.abs(_stg2MtrTargetRPM) >  0)
			{
				// if already turning, just bump
				RunStg2(_stg2MtrTargetRPM -= SHOOTER_BUMP_RPM);
				DriverStation.reportWarning("Bumping Up Stage 2", false);
			}
			else
			{
				// if currently stopped, set to default speed
				RunStg2(SECOND_STAGE_MTR_DEFAULT_RPM);
			}
		}
		else
		{
			DriverStation.reportWarning("Stg 2 Mtr Already at MAX ", false);
			_isStg2MtrTargetRPMBumpingUp = false;
		}
	}

	public void BumpStg2MtrRPMDown()
	{
		// only bump if not already at min
		if(Math.abs(_stg2MtrTargetRPM) > Math.abs(MIN_SHOOTER_RPM))
		{
			RunStg2(_stg2MtrTargetRPM += SHOOTER_BUMP_RPM);
		}
		else
		{
			DriverStation.reportWarning("Stg 2 Mtr Already at MIN ", false);
			_isStg2MtrTargetRPMBumpingUp = true;
		}
	}
	
	public void ControlHighSpeedLane()
	{
		// run High Speed Lane if both Shooter Motors are within 5% of non-zero cmd
		if(((Math.abs(_stg1MtrTargetRPM) > 0) && (Math.abs(getStg1RPMErrorPercent()) < 5))
				&& ((Math.abs(_stg2MtrTargetRPM) > 0) && (Math.abs(getStg2RPMErrorPercent()) < 5)))
		{
			RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
		} else
		{
			RunHighSpeedInfeedLane(0.0);
		}
	}
	
	private void RunHighSpeedInfeedLane(double percentVbusCommand)
	{
		_highSpeedInfeedLaneMtrTargetVBus = percentVbusCommand;
		_highSpeedInfeedLaneMtr.set(_highSpeedInfeedLaneMtrTargetVBus);
	}
	
	//============================================================================================
	// Run Magin Carpet / High Roller Motors
	//============================================================================================
	
	public void ToggleRunShooterFeeder()
	{
		// if current cmd is 0 or - if running in reverse, then start
		if(_magicCarpetMtrTargetVBus != MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND)
		{
			RunMagicCarpet(MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND);
			RunHighRoller(HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND);
		}
		else
		{
			FullShooterFeederStop();
		}
	}
	
	public void RunShooterFeederInReverse()
	{
		RunMagicCarpet(MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND * -1.0);
		RunHighRoller(HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND * -1.0);
	}
	
	public void CleanupRunShooterFeederInReverse()
	{
		if (_magicCarpetMtrTargetVBus == (MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND * -1.0))
		{
			FullShooterFeederStop();
		}
	}
	
	private void RunMagicCarpet(double percentVbusCommand)
	{
		_magicCarpetMtrTargetVBus = percentVbusCommand;
		_magicCarpetMtr.set(_magicCarpetMtrTargetVBus);
	}
	
	private void RunHighRoller(double percentVbusCommand)
	{
		_highRollerMtrTargetVBus = percentVbusCommand;
		_highRollerMtr.set(_highRollerMtrTargetVBus);
	}
	
	//============================================================================================
	// Linear Actuator
	//============================================================================================
	
	public void MoveActuatorToDefaultPosition()
	{
		_currentSliderPosition = INITIAL_POSITION_ACTUATOR;
		_linearActuator.setPosition(_currentSliderPosition);
		
		DriverStation.reportWarning("Actuator Configured to " + _currentSliderPosition, false);
	} 
	
	public void MoveActuatorUp()
	{
		// are we below the max?
		if (_currentSliderPosition < MAX_THRESHOLD_ACTUATOR)
		{
			// increment the target position
			_currentSliderPosition += CHANGE_INTERVAL_ACTUATOR;
			
			// protect fwd limit
			if(_currentSliderPosition > MAX_THRESHOLD_ACTUATOR) {
				_currentSliderPosition = MAX_THRESHOLD_ACTUATOR;
			}
			
			//rounds to 3 Decimal Places
			_currentSliderPosition = GeneralUtilities.RoundDouble(_currentSliderPosition, 3); 
			
			// actually move the slider
			_linearActuator.setPosition(_currentSliderPosition);
			
			DriverStation.reportWarning("Actuator Move Up To: " + _currentSliderPosition, false);
		}
		else
		{
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at MAXIMUM Position", false);
		}
	}
	
	public void MoveActuatorDown()
	{
		// are we above the min?
		if (_currentSliderPosition > MIN_THRESHOLD_ACTUATOR)
		{
			// decrement the target position
			_currentSliderPosition -= CHANGE_INTERVAL_ACTUATOR;
			
			// protect fwd limit
			if(_currentSliderPosition < MIN_THRESHOLD_ACTUATOR) {
				_currentSliderPosition = MIN_THRESHOLD_ACTUATOR;
			}
			
			//rounds to 3 Decimal Places
			_currentSliderPosition = GeneralUtilities.RoundDouble(_currentSliderPosition, 3); 
			
			// actually move the slider
			_linearActuator.setPosition(_currentSliderPosition);
			
			DriverStation.reportWarning("Actuator Move Down To: " + _currentSliderPosition, false);
		}
		else
		{
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at MINIMUM Position", false);
		}
	}
	
	//============================================================================================
	// Update Smart Dashboard with Current Values
	//============================================================================================	
	
	public void OutputToSmartDashboard()
	{
		//%s - insert a string
		//%d - insert a signed integer (decimal)
		//%f - insert a real number, standard notation
		
		// working varibles
		String stg1MtrData = "?";
		String stg2MtrData = "?";
		String shooterFeederMtrsData = "?";
		String actuatorData = "?";
		
		//Display Current Shooter Motor 1 & 2  | Target | Actual RPM | Error | %Out
		stg1MtrData = String.format("[%.0f RPM] %.0f RPM (%.2f%%) [%.2f%%]", 
												-1*_stg1MtrTargetRPM, 
												-1*getStg1ActualRPM(), 
												getStg1RPMErrorPercent(),
												getStg1CurrentPercentVBus());
		
		stg2MtrData = String.format("[%.0f RPM] %.0f RPM (%.2f%%) [%.2f%%]", 
												-1*_stg2MtrTargetRPM, 
												-1*getStg2ActualRPM(), 
												getStg2RPMErrorPercent(),
												getStg2CurrentPercentVBus());
		
		SmartDashboard.putString("Stg 1 RPM Target|Act|(Err%)|[%Out]", stg1MtrData);
		SmartDashboard.putString("Stg 2 RPM Target|Act|(Err%)|[%Out]", stg2MtrData);
		
		String magicCarpetMtrInOut = "?";
		if(_magicCarpetMtrTargetVBus == MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND) {
			magicCarpetMtrInOut = "in";
		} else if (_magicCarpetMtrTargetVBus == (MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND * -1.0)){
			magicCarpetMtrInOut = "out";
		} else if (_magicCarpetMtrTargetVBus == 0.0){
			magicCarpetMtrInOut = "off";
		}
		
		String highRollerMtrInOut = "?";
		if(_highRollerMtrTargetVBus == HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND) {
			highRollerMtrInOut = "in";
		} else if (_highRollerMtrTargetVBus == (HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND * -1.0)){
			highRollerMtrInOut = "out";
		} else if (_highRollerMtrTargetVBus == 0.0){
			highRollerMtrInOut = "off";
		}
		
		String highSpeedLaneMtrOnOff= "?";
		if(_highSpeedInfeedLaneMtrTargetVBus == HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND) {
			highSpeedLaneMtrOnOff = "on";
		} else if (_highSpeedInfeedLaneMtrTargetVBus == 0.0){
			highSpeedLaneMtrOnOff = "off";
		}
		
		//Display Current magicCarpet Infeed HighRoller MtrsData
		shooterFeederMtrsData = String.format("%.2f%% (%s) | %.2f%% (%s) | %.2f%% (%s)", 
																_magicCarpetMtrTargetVBus * 100.0,
																magicCarpetMtrInOut,
																_highRollerMtrTargetVBus * 100.0,
																highRollerMtrInOut,
																_highSpeedInfeedLaneMtrTargetVBus * 100.0,
																highSpeedLaneMtrOnOff);
		
		SmartDashboard.putString("MC | HR | HSL)", shooterFeederMtrsData);
		
		//Display Current Actuator Value
		actuatorData = String.format( "%.3f", _currentSliderPosition); //Outputs "Max" and "Min" at respective values
		
		if(_currentSliderPosition >= MAX_THRESHOLD_ACTUATOR)
		{
			actuatorData = actuatorData + " (MAX)";
		}
		else if(_currentSliderPosition <= MIN_THRESHOLD_ACTUATOR)
		{
			actuatorData = actuatorData + " (MIN)";
		}
		
		SmartDashboard.putString("Actuator Position", actuatorData);
	}
	
	//============================================================================================
	// Update Logging File
	//============================================================================================	
	
	public void UpdateLogData(LogData logData)
	{
		logData.AddData("Stg1Mtr:Cmd_Rpm", String.format("%f", _stg1MtrTargetRPM));
		logData.AddData("Stg1Mtr:Act_Rpm", String.format("%f", getStg1ActualRPM()));
		logData.AddData("Stg1Mtr:Err_%", String.format("%.2f%%", getStg1RPMErrorPercent()));
		logData.AddData("Stg1Mtr:%VBus", String.format("%.2f%%", getStg1CurrentPercentVBus()));
			
		logData.AddData("Stg2Mtr:Cmd_Rpm", String.format("%f", _stg2MtrTargetRPM));
		logData.AddData("Stg2Mtr:Act_Rpm", String.format("%f", getStg2ActualRPM()));	
		logData.AddData("Stg2Mtr:Err_%", String.format("%.2f%%", getStg2RPMErrorPercent()));
		logData.AddData("Stg2Mtr:%VBus", String.format("%.2f%%", getStg2CurrentPercentVBus()));

		logData.AddData("Actuator Position", String.format("%.3f", _currentSliderPosition));
	}
	
	//============================================================================================
	// PROPERTY ACCESSORS FOLLOW
	//============================================================================================
	
	// ---- Stage 1 ----------------------
	private double getStg1ActualRPM()
	{
		return _firstStgMtr.getSpeed();
	}
	
	private double getStg1RPMErrorPercent()
	{
		if(Math.abs(_stg1MtrTargetRPM) > 0 )
		{		
			return ((_stg1MtrTargetRPM - getStg1ActualRPM()) / _stg1MtrTargetRPM) * 100.0 * -1.0;
		}
		else
		{
			return 0.0;
		}
	}
	
	private double getStg1CurrentPercentVBus()
	{
		double currentOutputVoltage = _firstStgMtr.getOutputVoltage();
		double currentBusVoltage = _firstStgMtr.getBusVoltage();
		
		double currentActualSpeed = (currentOutputVoltage / currentBusVoltage);
		
		return GeneralUtilities.RoundDouble(currentActualSpeed, 2);
	}
	
	// ---- Stage 2 ----------------------
	private double getStg2ActualRPM()
	{
		return _secondStgMtr.getSpeed();
	}
	
	private double getStg2RPMErrorPercent()
	{
		if(Math.abs(_stg2MtrTargetRPM) > 0 )
		{		
			return ((_stg2MtrTargetRPM - getStg2ActualRPM()) / _stg2MtrTargetRPM) * 100.0 * -1.0;
		}
		else
		{
			return 0.0;
		}
	}
	
	private double getStg2CurrentPercentVBus()
	{
		double currentOutputVoltage = _secondStgMtr.getOutputVoltage();
		double currentBusVoltage = _secondStgMtr.getBusVoltage();
		
		double currentActualSpeed = (currentOutputVoltage / currentBusVoltage);
		
		return GeneralUtilities.RoundDouble(currentActualSpeed, 2);
	}
}