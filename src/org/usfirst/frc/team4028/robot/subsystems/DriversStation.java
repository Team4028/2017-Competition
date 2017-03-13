package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.utilities.LogData;

import edu.wpi.first.wpilibj.DriverStation;

//This class implements all functionality to read Operator & Driver Gamepads
//
//------------------------------------------------------
//	Rev		By		 	D/T				Desc
//	===		========	===========		=================================
//	0		TomB		17.Feb.2017		Initial Version
//	1		TomB		18.FEb.2018		added OPERATOR_START_BUTTON			Gear Tilt ReZero
//  2		SydneyS		11.Mar.2017		add POV buttons						Reassigned driver/operator buttons
//------------------------------------------------------
//
public class DriversStation extends BaseDriversStation {
	//============================================================================================
	// constructors follow
	//============================================================================================
	public DriversStation(int driverGamePadUsbPort, int operatorGamePadUsbPort, int engineeringGamePadUsbPort) {
		// call base class constructor
		super(driverGamePadUsbPort, operatorGamePadUsbPort, engineeringGamePadUsbPort);
	}
	
	/*
	==========================================================================
	--- Driver Joysticks --------
	DRIVER_LEFT_X_AXIS
	DRIVER_LEFT_Y_AXIS				ChassisThrottle_JoystickCmd
	DRIVER_LEFT_TRIGGER				Gear Infeed
	DRIVER_RIGHT_TRIGGER			Gear Outfeed
	DRIVER_RIGHT_X_AXIS				ChassisTurn_JoystickCmd
	DRIVER_RIGHT_Y_AXIS
	
	--- Driver Buttons --------
	DRIVER_GREEN_BUTTON_A		Gear Tilt Goto Floor
	DRIVER_RED_BUTTON_B			Gear Tilt Goto Score 
	DRIVER_BLUE_BUTTON_X		Hang Gear Sequence Initiation
	DRIVER_YELLOW_BUTTON_Y		Gear Tilt GoTo Home
	DRIVER_LEFT_BUMPER			Fuel Infeed
	DRIVER_RIGHT_BUMPER			Gear Shift Toggle
	DRIVER_BACK_BUTTON			Start Climb
	DRIVER_START_BUTTON			Gear Tilt ReZero
	DRIVER_LEFT_THUMBSTICK
	DRIVER_RIGHT_THUMBSTICK
	DRIVER_POV					Camera Swap
	
							
								Gear Tilt
							
	==========================================================================
		
	--- Operator Joysticks --------
	OPERATOR_LEFT_X_AXIS			
	OPERATOR_LEFT_Y_AXIS			
	OPERATOR_LEFT_TRIGGER			Chassis Spin Left
	OPERATOR_RIGHT_TRIGGER			Chassis Spin Right
	OPERATOR_RIGHT_X_AXIS
	OPERATOR_RIGHT_Y_AXIS			
	
	--- Operator Buttons --------
	OPERATOR_GREEN_BUTTON_A			Toggle Shooter Feeder Mtrs
	OPERATOR_RED_BUTTON_B			Shooter Slider Down
	OPERATOR_BLUE_BUTTON_X			Shoot Ball/Full Shooter Stop
	OPERATOR_YELLOW_BUTTON_Y		Shooter Slider Up
	OPERATOR_LEFT_BUMPER			Shooter Stg1 Cycle RPM
	OPERATOR_RIGHT_BUMPER			Shooter Stg2 Cycle RPM
	OPERATOR_BACK_BUTTON			High Speed Lane
	OPERATOR_START_BUTTON			Run Shooter Feeder Mtrs in Reverse
	OPERATOR_LEFT_THUMBSTICK		Shooter Stg1 Cycle Down RPM	
	OPERATOR_RIGHT_THUMBSTICK		Shooter Stg2 Cycle Down RPM
	OPERATOR_POV					Camera Swap
	==========================================================================
	*/
		
	// ======================================================
	// Public Property Accessors
	//		Implement calls into base class property accessors with 
	//		Robot function specific names
	// ======================================================
	
	// =========================================================================================================
	// DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER
	// =========================================================================================================
	
	// ===================================
	// === driver Just Pressed buttons ===
	// ===================================
	/*   <sample code>
	public boolean getIsDriver_FEATUREA_BtnJustPressed()
	{
		return super.getIsDriverGreenBtnAJustPressed();
	} 
	*/
		// Gear Tilt Floor
	public boolean getIsDriver_GearGoToFloor_BtnJustPressed() {
		return super.getIsDriverGreenBtnAJustPressed();
	}	
	
	// Gear Tilt Score 
	public boolean getIsDriver_GearGoToScore_BtnJustPressed() {
		return super.getIsDriverRedBtnBJustPressed();
	}
	// Gear AutoScore Sequence
	public boolean getIsDriver_GearStartSequence_BtnJustPressed() {
		return super.getIsDriverBlueBtnXJustPressed();
	}
	
	// Gear Tilt Home
	public boolean getIsDriver_GearGoToHome_BtnJustPressed() {
		return super.getIsDriverYellowBtnYJustPressed();
	}
	
	public boolean getIsDriver_StartClimb_ButtonJustPressed() {
		return super.getIsDriverBackBtnJustPressed();
	}

	// Gear ReZero
	public boolean getIsDriver_GearReZero_BtnJustPressed() {
		return super.getIsDriverStartBtnJustPressed();
	}
	
	// ConstantVelocityThroughVBus	
	public boolean getIsDriver_GearShiftToggle_BtnJustPressed() {
		return super.getIsDriverRightBumperBtnJustPressed();
	}
	
	// Toggle Camera
	public boolean getIsDriver_ToggleCamera_BtnJustPressed() {
		return super.getIsDriverPovUpBtnJustPressed();
	}
	
	// Camera Swap: Driver
	public boolean getIsDriver_CameraSwap_BtnJustPressed() {
		return super.getIsOperatorPovBtnJustPressed();
	}
	
	
	//ShooterStg1Up
	//public boolean getIsDriver_ShooterStg1Up_BtnJustPressed()
	//{
	//	return super.getIsDriverLeftBumperBtnJustPressed();
	//}
	
	//ShooterStg1Down
	//public boolean getIsDriver_ShooterStg1Down_BtnJustPressed()
	//{
	//	return super.getIsDriverBackBtnJustPressed();
	//}
	
	// Shooter Stg 1 Cycle Up/Down
	//public boolean getIsDriver_ShooterStg1CycleRPM_BtnJustPressed()
	//{
	//	return super.getIsDriverLeftBumperBtnJustPressed();
	//}

	
	// Blender Cycle
	//public boolean getIsDriver_BlenderCycleRPM_BtnJustPressed() {
	//	return super.getIsDriverStartBtnJustPressed();
	//}
	
	//ShooterStg2Down
	//public boolean getIsDriver_ShooterStg2Down_BtnJustPressed()
	//{
	//	return super.getIsDriverStartBtnJustPressed();
	//}
	
	// Shooter Stg 2 Cycle Up/Down
	//public boolean getIsDriver_ShooterStg2CycleRPM_BtnJustPressed()
	//{
	//	return super.getIsDriverRightBumperBtnJustPressed();
	//}
	
		
	// ===================================
	// === driver Is Pressed buttons =====
	// ===================================
	/*  <sample code>
	public boolean getIsDriver_FEATUREA_BtnPressed()
	{
		return super.getIsDriverGreenBtnPressed();
	}
	*/

	//Fuel Infeed
	public boolean getIsDriver_FuelInfeed_BtnPressed() {
		return super.getIsDriverLeftBumperBtnPressed();
	}
	
	public boolean getIsDriver_Climb_ButtonPressed() {
		return super.getIsDriverBackBtnPressed();
	}
	
	// ===================================
	// === driver Joysticks ==============
	// ===================================
	/*	<sample code>
	public double getDriver_FEATUREB_JoystickCmd()
	{
		return super.getDriverLeftXAxisCmd();
	}
	*/
	// Gear Infeed
	public double getDriver_GearInfeed_JoystickCmd() {
		return super.getDriverLeftTriggerCmd();
	}
	
	// Gear Outfeed
	public double getDriver_GearOutfeed_JoystickCmd() {
		return super.getDriverRightTriggerCmd();
	}
	
	// Chassis Throttle
	public double getDriver_ChassisThrottle_JoystickCmd() {
		return super.getDriverLeftYAxisCmd();
	}
	
	// Chassis Turn
	public double getDriver_ChassisTurn_JoystickCmd() {
		return super.getDriverRightXAxisCmd();
	}
	
	
	
	
	// =========================================================================================================
	// OPERATOR		OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	
	// =========================================================================================================
	
	
	// =====================================
	// === operator Just Pressed buttons ===
	// =====================================
	
	// Toggle Shooter Feeder Motors
	public boolean getIsOperator_ToggleShooterFeederMtrs_BtnJustPressed() {
		return super.getIsOperatorGreenBtnAJustPressed();
	}
	
	public boolean getIsOperator_ShooterStg1StepRPMUp_BtnJustPressed() {
		return super.getIsOperatorLeftBumperBtnJustPressed();
	}
	
	// Toggle Shoot Ball
	public boolean getIsOperator_ToggleShootBall_BtnJustPressed() {
		return super.getIsOperatorBlueBtnXJustPressed();
	}
	

	// Swap Cameras: Operator
	public boolean getIsOperator_CameraSwap_BtnJustPressed() {
		return super.getIsOperatorPovBtnJustPressed();
	}
	
	public boolean getIsOperator_ShooterStg2StepRPMUp_BtnJustPressed() {
		return super.getIsOperatorRightBumperBtnJustPressed();
	}
	
	//ActuatorUp
	public boolean getIsOperator_ShooterSliderUp_BtnJustPressed() {
		return super.getIsOperatorYellowBtnYJustPressed();
	}
	
	//ActuatorDown
	public boolean getIsOperator_ShooterSliderDown_BtnJustPressed() {
		return super.getIsOperatorRedBtnBJustPressed();
	}
	
	//Stg 1 Mtr Down
	public boolean getIsOperator_ShooterStg1StepRPMDown_BtnJustPressed() {
		return super.getIsOperatorLeftThumbstickBtnJustPressed();
	}
	
	// Stg 2 Mtr Down
	public boolean getIsOperator_ShooterStg2StepRPMDown_BtnJustPressed() {
		return super.getIsOperatorRightThumbstickBtnJustPressed();
	}
	
	//FullStop
	public boolean getIsOperator_HighSpeedLane_BtnJustPressed() {  
		return super.getIsOperatorBackBtnJustPressed();
	}
	
	// =====================================
	// === operator Is Pressed buttons =====
	// =====================================
	
	public boolean getIsOperator_RunShooterFeederInReverse_BtnPressed()
	{
		return super.getIsOperatorStartBtnPressed();
	}
	
	// =====================================
	// === operator Joysticks ==============
	// =====================================
	
	// Winch
	public double getOperator_Winch_JoystickCmd() {
		//return super.getOperatorRightYAxisCmd();
		return 0;
	}
	
	
	// GearInfeedOutFeed
	//public double getOperator_GearInfeedOutFeed_JoystickCmd() {
		//return super.getOperatorRightYAxisCmd();
	//}
	
	// TODO: Find a home for this button. Or does this button even need to exist? 11 Mar
	//public double getOperator_GearTiltFeed_JoystickCmd() {
		//return super.getOperatorLeftYAxisCmd();
	//}
	
	// chassis turret
	public double getOperator_ChassisSpinLeft_JoystickCmd() {
		return super.getOperatorLeftTriggerCmd();
	}

	public double getOperator_ChassisSpinRight_JoystickCmd() {
		return super.getOperatorRightTriggerCmd();
	}
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// update the Dashboard with any Drivers Station specific data values
	public void OutputToSmartDashboard() {	
	}
	
	public void UpdateLogData(LogData logData) {
	}
}