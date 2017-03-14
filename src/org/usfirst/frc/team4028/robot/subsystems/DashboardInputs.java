package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ALLIANCE;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.AUTON_MODE;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.CAMERA_NAMES;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality to read settings from the Dashboard
//
//------------------------------------------------------
//	Rev		By		 	D/T			Desc
//	===		========	===========	=================================
//	0		Sydney	 	15.Feb.2017	Initial Version
//	1		TomB		26.Feb.2017	Updated w/ new Auton Options
//  2 		Sebas		6.Mar.2017	Updated w/ more Auton Options
//  3       Nick        8.Mar.2017  Updated with Camera Options
//------------------------------------------------------
//
//=====> For Changes see Sydney
public class DashboardInputs { 
	
	private AUTON_MODE _autonModeChoice;
	private ALLIANCE _alliance;
	private CAMERA_NAMES _gearCameraName;
	private CAMERA_NAMES _shooterCameraName;
	private CAMERA_NAMES _climberCameraName;
	private CAMERA_NAMES _driverCameraName;
	
	private SendableChooser<AUTON_MODE> _autonModeChooser;
	private SendableChooser<ALLIANCE> _allianceChooser;
	private SendableChooser<CAMERA_NAMES> _gearCamChooser;
	private SendableChooser<CAMERA_NAMES> _shooterCamChooser;
	private SendableChooser<CAMERA_NAMES> _climberCamChooser;
	private SendableChooser<CAMERA_NAMES> _driverCamChooser;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public DashboardInputs() {
		ConfigAutonModeChoosers();
		ConfigCameraChoosers();
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	private void ConfigAutonModeChoosers() {
		//============================
		// Autonomous Mode Choice
		//============================
		_autonModeChooser = new SendableChooser<AUTON_MODE>();
		
		_autonModeChooser.addObject("Do Nothing", GeneralEnums.AUTON_MODE.DO_NOTHING);
		_autonModeChooser.addObject("Cross the Base Line", GeneralEnums.AUTON_MODE.CROSS_BASE_LINE);
		_autonModeChooser.addObject("Hang Gear on Boiler Side", GeneralEnums.AUTON_MODE.HANG_BOILER_GEAR);
		_autonModeChooser.addObject("Hang Gear on the Boiler Side and Shoot", GeneralEnums.AUTON_MODE.HANG_BOILER_GEAR_AND_SHOOT);
		_autonModeChooser.addDefault("Hang Gear in Center", GeneralEnums.AUTON_MODE.HANG_CENTER_GEAR);
		_autonModeChooser.addObject("Hang Gear in Center and Shoot", GeneralEnums.AUTON_MODE.HANG_CENTER_GEAR_AND_SHOOT);
		_autonModeChooser.addObject("Hang Gear on Retrieval Side", GeneralEnums.AUTON_MODE.HANG_RETRIEVAL_GEAR);
		_autonModeChooser.addObject("Hit the Hopper and Shoot", GeneralEnums.AUTON_MODE.HIT_HOPPER);
		_autonModeChooser.addObject("Turn and Shoot", GeneralEnums.AUTON_MODE.TURN_AND_SHOOT);
		_autonModeChooser.addObject("Two Gears", GeneralEnums.AUTON_MODE.TWO_GEAR);
		
		SmartDashboard.putData("Auton Mode Chooser", _autonModeChooser);
		
		//============================
		// Alliance Choice
		//============================
		_allianceChooser = new SendableChooser<ALLIANCE>();
		
		_allianceChooser.addDefault("Red Alliance", GeneralEnums.ALLIANCE.RED_ALLIANCE);
		_allianceChooser.addObject("Blue Alliance", GeneralEnums.ALLIANCE.BLUE_ALLIANCE);
		
		SmartDashboard.putData("Alliance Chooser" , _allianceChooser);		
	}
	
	private void ConfigCameraChoosers()
	{
		//====================================
		//#ALL the Cameras
		//=====================================
		_gearCamChooser = new SendableChooser<CAMERA_NAMES>();
		
		_gearCamChooser.addDefault("g.cam0", GeneralEnums.CAMERA_NAMES.CAM0);
		_gearCamChooser.addObject("cam1", GeneralEnums.CAMERA_NAMES.CAM1);
		_gearCamChooser.addObject("cam2", GeneralEnums.CAMERA_NAMES.CAM2);
		_gearCamChooser.addObject("cam3", GeneralEnums.CAMERA_NAMES.CAM3);
		
		SmartDashboard.putData("Gear Camera Chooser", _gearCamChooser);
		
		
		_shooterCamChooser = new SendableChooser<CAMERA_NAMES>();
		
		_shooterCamChooser.addObject("s.cam0", GeneralEnums.CAMERA_NAMES.CAM0);
		_shooterCamChooser.addDefault("cam1", GeneralEnums.CAMERA_NAMES.CAM1);
		_shooterCamChooser.addObject("cam2", GeneralEnums.CAMERA_NAMES.CAM2);
		_shooterCamChooser.addObject("cam3", GeneralEnums.CAMERA_NAMES.CAM3);
		
		SmartDashboard.putData("Shooter Camera Chooser", _shooterCamChooser);
		
		
		
		_climberCamChooser = new SendableChooser<CAMERA_NAMES>();
		
		_climberCamChooser.addObject("c.cam0", GeneralEnums.CAMERA_NAMES.CAM0);
		_climberCamChooser.addObject("cam1", GeneralEnums.CAMERA_NAMES.CAM1);
		_climberCamChooser.addDefault("cam2", GeneralEnums.CAMERA_NAMES.CAM2);
		_climberCamChooser.addObject("cam3", GeneralEnums.CAMERA_NAMES.CAM3);
		
		SmartDashboard.putData("Climber Camera Chooser", _climberCamChooser);
		
		
		
		_driverCamChooser = new SendableChooser<CAMERA_NAMES>();
		
		_driverCamChooser.addObject("d.cam0", GeneralEnums.CAMERA_NAMES.CAM0);
		_driverCamChooser.addObject("cam1", GeneralEnums.CAMERA_NAMES.CAM1);
		_driverCamChooser.addObject("cam2", GeneralEnums.CAMERA_NAMES.CAM2);
		_driverCamChooser.addDefault("cam3", GeneralEnums.CAMERA_NAMES.CAM3);
		
		SmartDashboard.putData("Driver Camera Chooser", _driverCamChooser);
	}

	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	public AUTON_MODE get_autonMode() {
		_autonModeChoice = _autonModeChooser.getSelected();
		return _autonModeChoice;
	}
	
	public ALLIANCE get_allianceMode() {
		_alliance =  _allianceChooser.getSelected();
		return _alliance;
	}
	
	public CAMERA_NAMES get_gearCam()
	{
		_gearCameraName = _gearCamChooser.getSelected();
		return _gearCameraName;
	}
	
	public CAMERA_NAMES get_shooterCam()
	{
		_shooterCameraName = _shooterCamChooser.getSelected();
		return _shooterCameraName;
	}
	
	public CAMERA_NAMES get_climberCam()
	{
		_climberCameraName = _climberCamChooser.getSelected();
		return _climberCameraName;
	}
	
	public CAMERA_NAMES get_driverCam()
	{
		_driverCameraName = _driverCamChooser.getSelected();
		return _driverCameraName;
	}
}
