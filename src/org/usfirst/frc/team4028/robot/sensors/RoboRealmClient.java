package org.usfirst.frc.team4028.robot.sensors;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Date;
import java.util.TimerTask;
import java.util.Vector;

import org.opencv.core.Mat;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.VISION_CAMERAS;
import org.usfirst.frc.team4028.robot.utilities.LogData;
import org.usfirst.frc.team4028.robot.vision.Dimension;
import org.usfirst.frc.team4028.robot.vision.RoboRealmAPI;
import org.usfirst.frc.team4028.robot.vision.Utilities;
import org.usfirst.frc.team4028.robot.vision.RawImageData;
import org.usfirst.frc.team4028.robot.subsystems.DashboardInputs;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is a wrapper around the interface to RoboRealm over TCP Sockets hosted on a 
 * on-robot Kangaroo
 * 
 *     	//_roboRealmClient.ChangeToCamera(ViSION_CAMERAS.BOILER);
 */
public class RoboRealmClient {
	// Define local working variables
 	private RoboRealmAPI _rrAPI;
 	
	//private java.util.Timer _updaterTimer; 
 	//private RoboRealmUpdater _task; 

 	private boolean _isConnected;
 	private String _currentVisionCameraName;
 	
 	private static final int TARGET_MINIMUM_Y_VALUE = 413;
 	
 	private static final int SOUTHWEST_X_IDX = 0;
 	private static final int SOUTHWEST_Y_IDX = 1;
 	private static final int SOUTHEAST_X_IDX = 2;
 	private static final int SOUTHEAST_Y_IDX = 3;
 	private static final int HIGH_MIDDLE_Y_IDX = 4;
 	private static final int  RAW_WIDTH_IDX = 5;
 	private static final int  RAW_HEIGHT_IDX = 6;
 	private static final int BLOB_COUNT_IDX = 7;
 	private static final int CAMERA_TYPE_IDX = 8;
 	
 	private static final int BAD_DATA_COUNTER_THRESHOLD = 10;
 	private static final int POLLING_CYCLE_IN_MSEC = 50; //100;

 	private static final int EXPECTED_ARRAY_SIZE = 9;
 	private static final int EXPECTED_GEAR_BLOB_COUNT = 2;
 	private static final int EXPECTED_BOILER_BLOB_COUNT = 1;
 	
 	private static final double CAMERA_FOV_HORIZONTAL_DEGREES = 83.0; // 58.5;
 	
 	// =============================================================
 	// Camera Adjustment Factor
 	// =============================================================
 	private static final double GEAR_CAMERA_CALIBRATION_FACTOR = 0.0;
 	private static final double BOILER_CAMERA_CALIBRATION_FACTOR = 0.0;
 	
 	private static final double CAMERA_ANGLE_IN_DEGREES = 38.38;	// <====== SET ME BASED ON CALIB PROCEDURE ============
 	
 	private double _cameraCalibrationFactor = GEAR_CAMERA_CALIBRATION_FACTOR;
 	// =============================================================
 	
 	
 	// working variables raised to class level to reduce GC pressure inside update task
 	private Dimension _fovDimensions;
 	private Vector _vector;
 	private long _callElapsedTimeMSec;
 	private long _lastCallTimestamp;
 	private RawImageData _newTargetRawData;
 	private double _fovXCenterPoint;
 	private double _targetXCenterPoint;
 	private double _fovCenterToTargetXAngleRawDegrees;
 	private int _badDataCounter;
 	private long _lastDebugWriteTimeMSec;
 	private final Object _targetDataMutex;
 	private int _expectedBlobCount;
 	private double _degreesFromCenter;
 	//private Solenoid _gearCamLED;
 	//private Solenoid _shooterCamLED;
 	DigitalOutput _visionLedsRelay;
 	
	//============================================================================================
	// constructors follow
	//============================================================================================
    public RoboRealmClient(String kangarooIPv4Addr, int rrPortNo, int visioLEDsDIOPort)
    		//int PCMCanAddr, int gearCamLEDPCMPort, int shooterCamLEDPCMPort) 
    {        	
    	// create an instance of the RoboRealm API client
    	_rrAPI = new RoboRealmAPI();
    	
    	//Utilities.SimplePingTest(kangarooIPv4Addr);
    	Utilities.RobustPortTest(kangarooIPv4Addr, rrPortNo);
    	
    	// try to connect
        if (!_rrAPI.connect(kangarooIPv4Addr, rrPortNo)) {
        	DriverStation.reportError("Could not connect to RoboRealm!", false);
        	System.out.println("====> Could not connect to RoboRealm!");
        	
        	_isConnected = false;
        } else {
        	DriverStation.reportWarning("Connected to RoboRealm!", false);
        	System.out.println("====> Connected to RoboRealm!");
        	
        	_isConnected = true;
        }
        
		// create instance of the task the timer interrupt will execute
		//_task = new RoboRealmUpdater();
		
		// create a timer to fire events
		_lastCallTimestamp = new Date().getTime();
		//_updaterTimer = new java.util.Timer();
		//_updaterTimer.scheduleAtFixedRate(_task, 0, POLLING_CYCLE_IN_MSEC);
		
		// start the camera thread
		Thread visionThread = new Thread(RoboRealmUpdater);
		visionThread.start();
		
		//Initialize counter to indicate bad data until proved good
		_badDataCounter = 10;
		
		_targetDataMutex = new Object();
		
		// relay: https://wpilib.screenstepslive.com/s/4485/m/13809/l/599706-on-off-control-of-motors-and-other-mechanisms-relays
		//TODO: create the relay object
		//Set up LED PCM Rings
		//_gearCamLED = new Solenoid(PCMCanAddr, gearCamLEDPCMPort);
		//_shooterCamLED = new Solenoid(PCMCanAddr, shooterCamLEDPCMPort);
		//_visionLedsRelay = new DigitalOutput(visioLEDsDIOPort);
		//TurnAllVisionLEDsOff();
    }
    
	//============================================================================================
	// Methods follow
	//============================================================================================
    public void TestConnect() {
    	Dimension fovDimensions = _rrAPI.getDimension();
    }
    
    public void TurnAllVisionLEDsOff()
    {
    	//_visionLedsRelay.set(false);
		//_gearCamLED.set(false);
		//_shooterCamLED.set(false);
    }
    
    public void TurnGearVisionLEDsOn()
    {
		//_shooterCamLED.set(false);
		//try {
		//	Thread.sleep(2);
		//} catch (InterruptedException e) {
			// TODO Auto-generated catch block
		//	e.printStackTrace();
		//}
		//_gearCamLED.set(true);
    	//_visionLedsRelay.set(true);
    }
    
    public void TurnBoilerrVisionLEDsOn()
    {
    	//_gearCamLED.set(false);
		//try {
		//	Thread.sleep(2);
		//} catch (InterruptedException e) {
			// TODO Auto-generated catch block
		//	e.printStackTrace();
		//}
		//_shooterCamLED.set(true);
    	//_visionLedsRelay.set(true);
    }
    
    // this method changes the active roborealm camera / program
    public void ChangeToCamera(VISION_CAMERAS visionCamera) {
    	_currentVisionCameraName = visionCamera.get_cameraName();
    	
    	switch(visionCamera) {
	    	case GEAR:
	        	_cameraCalibrationFactor = GEAR_CAMERA_CALIBRATION_FACTOR;
	        	_expectedBlobCount = EXPECTED_GEAR_BLOB_COUNT;
	        	//_gearCamLED.set(true);
	        	//_shooterCamLED.set(false);
	        	//_visionLedsRelay.set(true);
	        	
	    		break;
	    		
	    	case BOILER:
	        	_cameraCalibrationFactor = BOILER_CAMERA_CALIBRATION_FACTOR;
	        	_expectedBlobCount = EXPECTED_BOILER_BLOB_COUNT;
	        	//_gearCamLED.set(false);
	        	//_shooterCamLED.set(true);
	        	//_visionLedsRelay.set(true);
	    		break;
    	}
    	
    	if (_rrAPI.setVariable("CamType", _currentVisionCameraName)) {
    		System.out.println("====> RoboRealm Camera switched to " + _currentVisionCameraName);
    	}
    	else {
    		System.out.println("====> FAILED changing RoboRealm Camera to " + _currentVisionCameraName);
    	}
    }

    
 	// this method switches the currently running pipeline program
 	public void SwitchProgram(String pipeLineProgramFullPathName) {
 		Boolean isPauseOk = _rrAPI.pause();
 		
 		System.out.println("====> Prepare to Switch Program");
 		
 		if(isPauseOk) {
        	//DriverStation.reportError("RoboRealm Program Paused successfully!", false);
        	System.out.println("====> RoboRealm Program Paused succesfully!");
        } else {
        	//DriverStation.reportError("Error..Could not pause RoboRealm program!", false);
        	System.out.println("====>Error..Could not pause RoboRealm program!");
        }
 		
 		System.out.println("====> Chg Program To: " + pipeLineProgramFullPathName);
 		
 		Boolean isSwitchOk = _rrAPI.loadProgram(pipeLineProgramFullPathName);
 		
 		if(isSwitchOk) {
        	//DriverStation.reportError("RoboRealm Program Switched succesfully to: " + pipeLineProgramFullPathName, false);
        	System.out.println("====> RoboRealm Program Switched succesfully to: " + pipeLineProgramFullPathName);
        } else {
        	//DriverStation.reportError("Error..Could not switch RoboRealm program!", false);
        	System.out.println("====> Error..Could not switch RoboRealm program!");
        }	
 		
 		Boolean isResumeOk = _rrAPI.resume();
 		
 		if(isResumeOk) {
        	//DriverStation.reportError("RoboRealm Program Paused succesfully!", false);
        	System.out.println("====> RoboRealm Program Resumed succesfully!");
        } else {
        	//DriverStation.reportError("Error..Could not pause RoboRealm program!", false);
        	System.out.println("====> Error..Could not resume RoboRealm program!");
        }
 	}
 	
	// update the Dashboard with any Vision specific data values
	public void OutputToSmartDashboard() {
		String dashboardMsg = "";
		
		if( get_isVisionDataValid()){
			synchronized (_targetDataMutex) {
				dashboardMsg = "Camera= " + _newTargetRawData.CameraType
								+ " Angle= " + _fovCenterToTargetXAngleRawDegrees 
								+ " MidHiY= " +  _newTargetRawData.HighMiddleY
								+ " mSec=" + _newTargetRawData.ResponseTimeMSec
								+ " Blob Count= " + _newTargetRawData.BlobCount 
								+ " Is on Gear Target= " + Boolean.toString(get_isInGearHangPosition());
				
			}
		} else {
			dashboardMsg = "Vision DATA NOT VALID";
		}
		
		SmartDashboard.putString("VIsion", dashboardMsg);
	} 	
	
	public void UpdateLogData(LogData logData) {
		synchronized (_targetDataMutex) {
			logData.AddData("RR:Camera", String.format("%s", _currentVisionCameraName.toString()));
			logData.AddData("RR:Angle", String.format("%.2f", _fovCenterToTargetXAngleRawDegrees));
			if(_newTargetRawData != null)
			{
				logData.AddData("RR:HiMidY", String.format("%.2f", _newTargetRawData.HighMiddleY));	
			}
			else {
				logData.AddData("RR:HiMidY", "N/A");	
			}
				
		}
	}
	
	//=========================================================================
	//	Task Executed By Timer
	//=========================================================================	
 	
	// poll RoboRealm to read current values
 	public void update() { 
 	    synchronized (_targetDataMutex) {
	 		
	 		// get the Field Of View Dimensions
	 		//_fovDimensions = _rrAPI.getDimension();
	 		
	 	    // get multiple variables
	 		// This must match what is in the config of the "Point Location" pipeline step in RoboRealm
	 	    //_vector = _rrAPI.getVariables("SW_X,SW_Y,SE_X,SE_Y,SCREEN_WIDTH,SCREEN_HEIGHT,BLOB_COUNT");
	 		_vector = _rrAPI.getVariables("SW_X,SW_Y,SE_X,SE_Y,HI_MID_Y,SCREEN_WIDTH,SCREEN_HEIGHT,BLOB_COUNT,CamType");
	 	    _callElapsedTimeMSec = new Date().getTime() - _lastCallTimestamp;
	 	    _newTargetRawData = null;
	 	    
	 	    if (_vector==null) {
	 	    	// write 1st 10 errors then every 50
	 	    	if(_badDataCounter <= 10 || _badDataCounter % 50 == 0) {
	 	    		System.out.println("Error in GetVariables, did not return any results [" + _badDataCounter + "]");
	 	    	}
	 	    	
	 	    	//Increment bad data counter
	 	    	_badDataCounter += 1;
	 	    }
	 	    else if(_vector.size() == EXPECTED_ARRAY_SIZE) {
	 	    	_newTargetRawData = new RawImageData();
	 	    	
	 	    	// parse the results and build the image data
	 	    	_newTargetRawData.Timestamp = new Date().getTime();
	 	    	_newTargetRawData.CameraType = (String)_vector.elementAt(CAMERA_TYPE_IDX);	    	
	 	    	_newTargetRawData.SouthWestX = Double.parseDouble((String)_vector.elementAt(SOUTHWEST_X_IDX));
	 	    	_newTargetRawData.SouthWestY = Double.parseDouble((String)_vector.elementAt(SOUTHWEST_Y_IDX));
	 	    	_newTargetRawData.SouthEastX = Double.parseDouble((String)_vector.elementAt(SOUTHEAST_X_IDX));
	 	    	_newTargetRawData.SouthEastY = Double.parseDouble((String)_vector.elementAt(SOUTHEAST_Y_IDX));
	 	    	_newTargetRawData.HighMiddleY = Double.parseDouble((String)_vector.elementAt(HIGH_MIDDLE_Y_IDX));
	 	    	
	 	    	_newTargetRawData.BlobCount = Integer.parseInt((String)_vector.elementAt(BLOB_COUNT_IDX));
	 	    	
	 	    	_fovDimensions = new Dimension();
	 	    	_fovDimensions.width = Double.parseDouble((String)_vector.elementAt(RAW_WIDTH_IDX));
	 	    	_fovDimensions.height = Double.parseDouble((String)_vector.elementAt(RAW_HEIGHT_IDX));
	 	    	_newTargetRawData.FOVDimensions = _fovDimensions;
	 	    	
	 	    	double estDistance = CalcDistanceUsingPolynomial(_newTargetRawData.HighMiddleY);
	 	    	double estDistance2 = CalcDistanceUsingCameraAngle(_newTargetRawData.HighMiddleY);
	 	    	
	 	    	_newTargetRawData.EstimatedDistance = estDistance;
	 	    	
	 	    	_newTargetRawData.ResponseTimeMSec = _callElapsedTimeMSec; 	    	
	 	
	 	    	// debug
	 	    	/*
	 	    	System.out.println("FOVh: " + _fovDimensions.height 
									+ " FOVw: " + _fovDimensions.width 
									+ " SWx: " + _newTargetRawData.SouthWestX 
	 	    						+ " SWy: " + _newTargetRawData.SouthWestY 
	 	    						+ " SEx: " + _newTargetRawData.SouthEastX 
	 	    						+ " SEy: " + _newTargetRawData.SouthEastY
	 	    						+ " mSec: " + _newTargetRawData.ResponseTimeMSec);    	
	 	    	*/
	 	    	
	 	    	// calc the horiz center of the image
	 	    	_fovXCenterPoint = _fovDimensions.width / 2.0;
	 	    	
	 	    	// calc the target center point
	 	    	_targetXCenterPoint = Math.round(((_newTargetRawData.SouthEastX + _newTargetRawData.SouthWestX) / 2.0) + _cameraCalibrationFactor);
	 	    	
	 	    	_fovCenterToTargetXAngleRawDegrees = ((_fovXCenterPoint - _targetXCenterPoint) * CAMERA_FOV_HORIZONTAL_DEGREES) / _fovDimensions.width;
	 	    	
	 	    	// round to 2 decimal places
	 	    	_fovCenterToTargetXAngleRawDegrees = Math.round(_fovCenterToTargetXAngleRawDegrees * 100) / 100.0;	
	 	    	
	 	    	// limit spamming
	 	    	if((new Date().getTime() - _lastDebugWriteTimeMSec) > 1000) {
		    		System.out.println("Vision Data Valid? " + get_isVisionDataValid()
		    							+ " |Camera= " + _newTargetRawData.CameraType 
		    							+ " |Angle= " + _fovCenterToTargetXAngleRawDegrees 
		    							+ " |HiMidY= " + _newTargetRawData.HighMiddleY
		    							+ " |DistInches= " + _newTargetRawData.EstimatedDistance
		    							+ " |DistInchesCA= " + estDistance2
		    							+ " |mSec=" + _newTargetRawData.ResponseTimeMSec
		    							+ " |BlobCnt= " + _newTargetRawData.BlobCount 
		    							+ " |IsGearOnTarget= " + Boolean.toString(get_isInGearHangPosition()));
		    		// reset last time
		    		_lastDebugWriteTimeMSec = new Date().getTime();
	 	    	}
	 	    	
	 	    	//Reset the counter 
	 	    	_badDataCounter = 0;
	 	    	_lastCallTimestamp = new Date().getTime();
	 	    } else {
	 	    	if((new Date().getTime() - _lastDebugWriteTimeMSec) > 1000) {
		 	    	System.out.println("Unexpected Array Size: " + _vector.size());
		    		// reset last time
		    		_lastDebugWriteTimeMSec = new Date().getTime();
	 	    	}
	 	    	
	 	    	_newTargetRawData = null;
	
	 	    	//Increment bad data counter
	 	    	_badDataCounter += 1;
	 	    }
 	    }
 	    

 	} 
 	
 	// this method used a n order polynomial to estimate the distance based on data collected using a specific, fixed camera angle approx 40 degrees
 	//	the polynomial coefficients will not be correct for other angles
    private double CalcDistanceUsingPolynomial(double highMiddleYInPixels)
    {
    	//=((-1.724236*10^-7) * A26^3) + ((1.6430741*10^-4) * A26^2) - (0.06984136 * A26) + 18.45576757
    	//double estDistance =(-1.724236 * Math.pow(10, -7) * Math.pow(_newTargetRawData.HighMiddleY, 3)) 
    	//						+ (1.6430741 * Math.pow(10, -4) * Math.pow(_newTargetRawData.HighMiddleY, 2)) 
    	//						+ (-0.06984136 * _newTargetRawData.HighMiddleY) 
    	//						+ 18.45576757;
    	
    	// inches = -0.00005952 pixels^3 + 0.04060523pixels^2 - 10.34270244pixels + 963.84165834
    	//double estDistance = (-0.00005952 * Math.pow(_newTargetRawData.HighMiddleY, 3)) 
    	//						+ (0.04060523 * Math.pow(_newTargetRawData.HighMiddleY, 2))
    	//						+ (-10.34270244 * _newTargetRawData.HighMiddleY) 
    	//						+ 963.84165834;
    	
    	// charlie's original numbers to front of can
    	// inches= -0.00000207pixels^3 + 0.00209429pixels^2 - 0.91180787pixels + 235.66718419
    	//double estDistance = (-0.00000207 * Math.pow(_newTargetRawData.HighMiddleY, 3))
    	//						+ (0.00209429 * Math.pow(_newTargetRawData.HighMiddleY, 2))
    	//						+ (-0.91180787 * _newTargetRawData.HighMiddleY)
    	//						+ 235.66718419;
    	
    	// charlie's numbers to center of can
    	// inches= -0.00005952pixels^3 + 0.04198913pixels^2 - 10.982809pixels + 1046.4642
    	double estDistanceInInches = (-0.000001 * Math.pow(_newTargetRawData.HighMiddleY, 3))
		    							+ (0.001388 * Math.pow(_newTargetRawData.HighMiddleY, 2))
		    							+ (-0.783982 * _newTargetRawData.HighMiddleY)
		    							+ 239.332871;
    	
    	return estDistanceInInches;
    }
 	
    // this method uses the actual camera angle to estimate the distance
    private double CalcDistanceUsingCameraAngle(double highMiddleYInPixels)
    {
    	final double HALF_FIELD_OF_VIEW_IN_DEGREES = 23.5645549; // 20.596
    	final double TARGET_HEIGHT_IN_INCHES = 87.875; // 87.875
//    	final double SENSOR_HEIGHT_IN_INCHES = 0.09877;	// 0.09877
    	final double SENSOR_HEIGHT_PIXELS = 479.115486; // 476.35
    	final double FOCAL_LENGTH_IN_INCHES = 0.14362022; // 0.13973
    	final double OFFSET_0_IN_INCHES = 9.99997139; // 1.807
    	final double OFFSET_1_IN_INCHES = 0.1409929; // 0.0109

    	double sensorHeightInInches = Math.tan(Math.toRadians(HALF_FIELD_OF_VIEW_IN_DEGREES)) * FOCAL_LENGTH_IN_INCHES * 2.0;
    	
    	double cameraAngleInRadians = Math.toRadians(CAMERA_ANGLE_IN_DEGREES);
    	
    	double relativeTargetHeight = TARGET_HEIGHT_IN_INCHES 
    									- 16.965
    									+ (0.925 * Math.cos(Math.toRadians(19.75 + CAMERA_ANGLE_IN_DEGREES)));
    	
    	double x3 = (FOCAL_LENGTH_IN_INCHES * Math.tan(cameraAngleInRadians))
    					- (sensorHeightInInches / 2.0)
    					+ ((highMiddleYInPixels * sensorHeightInInches) / SENSOR_HEIGHT_PIXELS);
    	
    	double estDistanceInInches = (
    								  ( (FOCAL_LENGTH_IN_INCHES / (x3 * Math.pow(Math.cos(cameraAngleInRadians), 2)) - (Math.tan(cameraAngleInRadians)) )
    									* relativeTargetHeight
    									* (1 - OFFSET_1_IN_INCHES)
    									)
    									- OFFSET_0_IN_INCHES
    									+ (29.75 + (0.925 * Math.sin(Math.toRadians(19.75 + CAMERA_ANGLE_IN_DEGREES))))
    								  );
    	
    	return estDistanceInInches;
    }
    
    // this is the task run by the timer
    /*private class RoboRealmUpdater extends TimerTask { 
 		public void run() { 
 			while(true) { 
 				update(); 
 				
 				try { 
 					// sleep for 10 mSec
 					Thread.sleep(10); 
				} catch (InterruptedException e) { 
 					e.printStackTrace(); 
				} 
 			} 
 		} 
 	}*/
    
    // =========================================================
    // Property Accessors
    // =========================================================
 	public double get_Angle() { 
 		return _fovCenterToTargetXAngleRawDegrees; 
 	}
    
    public double get_fovCenterToTargetXAngleRawDegrees() {
    	return _fovCenterToTargetXAngleRawDegrees;
    }
    
    public boolean get_isVisionDataValid() {
    	synchronized (_targetDataMutex) {
	    	if((_badDataCounter < BAD_DATA_COUNTER_THRESHOLD) 
	    			&& (_newTargetRawData != null)
	    			&& (_newTargetRawData.BlobCount == _expectedBlobCount)) {	
	    		return true;
	    	} else {
	    		return false;
	    	}
    	}
    }
    
    public double get_DistanceToBoilerInches() {
    	if(get_isVisionDataValid()) {
    		return _newTargetRawData.EstimatedDistance;
    	}
    	else {
    		return 0.0;
    	}
    }
    
    public boolean get_isInGearHangPosition() {
		if (get_isVisionDataValid()
				&& (_newTargetRawData.SouthEastY > TARGET_MINIMUM_Y_VALUE) 
				&& (_newTargetRawData.SouthWestY > TARGET_MINIMUM_Y_VALUE)) {
			return true;
		} else {
			return false;
		}
    }
    
    //================================================================================================
	private Runnable RoboRealmUpdater = new Runnable() {
		@Override
		public void run() {   
            
            // =============================================================================
            // start looping
            // =============================================================================
            while(!Thread.interrupted()) {            	
            	update();
            	        		
        		// sleep each cycle to avoid Robot Code not updating often enough issues
        		try {
					Thread.sleep(POLLING_CYCLE_IN_MSEC);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
            }
	            	
		}
	};
}