package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ALLIANCE;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.AUTON_MODE;

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
//------------------------------------------------------
//
//=====> For Changes see Sydney
public class DashboardInputs { 
	
	private AUTON_MODE _autonModeChoice;
	private ALLIANCE _alliance;
	
	private SendableChooser<AUTON_MODE> _autonModeChooser;
	private SendableChooser<ALLIANCE> _allianceChooser;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public DashboardInputs() {
		ConfigAutonModeChoosers();
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
}
