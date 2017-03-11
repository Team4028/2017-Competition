package org.usfirst.frc.team4028.robot.constants;

// this class contains general enums
public class GeneralEnums {
	public enum TELEOP_MODE {
		STANDARD,
		HANG_GEAR_SEQUENCE_MODE,
		CLIMBING,
		AUTO_AIM
	}
	
	public enum AUTON_MODE {
		UNDEFINED,
		CROSS_BASE_LINE,
		DO_NOTHING,
		HANG_BOILER_GEAR,
		HANG_BOILER_GEAR_AND_SHOOT,
		HANG_CENTER_GEAR,
		HANG_CENTER_GEAR_AND_SHOOT,
		HANG_RETRIEVAL_GEAR,
		HIT_HOPPER,
		TURN_AND_SHOOT,
		TWO_GEAR
	}
	
	public enum ALLIANCE {
		BLUE_ALLIANCE,
		RED_ALLIANCE
	}
	
	public enum MOTION_PROFILE {
		BOILER_GEAR,
		CENTER_GEAR,
		HOPPER_TO_SHOOTING_POSITION,
		MOVE_TO_BOILER,
		MOVE_TO_HOPPER,  
		RETRIEVAL_GEAR,
		TURN_AND_SHOOT,
		TWO_GEAR_LONG,
		TWO_GEAR_SHORT_FWD,
		TWO_GEAR_SHORT_REV,
		TWO_GEAR_SUPER_SHORT
	}
	
	public enum CAMERA_NAMES
	{
		UNDEFINED 	("N/A"),
		CAM0		("cam0"),
		CAM1		("cam1"),
		CAM2		("cam2"),
		CAM3		("cam3");
		
		private final String _cameraName;  
		
		CAMERA_NAMES(String cameraName)
		{
			_cameraName = cameraName;
		}
		
		public String get_cameraName()
		{
			return _cameraName;
		}
	}
}
