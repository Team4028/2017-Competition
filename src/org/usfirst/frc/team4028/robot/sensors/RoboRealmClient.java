package org.usfirst.frc.team4028.robot.sensors;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Date;
import java.util.TimerTask;
import java.util.Vector;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ViSION_CAMERAS;
import org.usfirst.frc.team4028.robot.vision.Dimension;
import org.usfirst.frc.team4028.robot.vision.RoboRealmAPI;
import org.usfirst.frc.team4028.robot.vision.Utilities;
import org.usfirst.frc.team4028.robot.vision.RawImageData;

//import uk.co.geolib.geopolygons.*;
//import uk.co.geolib.geolib.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is a wrapper around the interface to RoboRealm over TCP Sockets hosted on a 
 * on-robot Kangaroo
 * 
 *     	//_roboRealmClient.ChangeToCamera(ViSION_CAMERAS.BOILER);
 */
public class RoboRealmClient 
{
	// Define local working variables
 	private RoboRealmAPI _rrAPI;
 	
	private java.util.Timer _updaterTimer; 
 	private RoboRealmUpdater _task; 

 	private boolean _isConnected;
 	private String _currentVisionCameraName;
 	
 	private static final int TARGET_MINIMUM_Y_VALUE = 413;
 	private static final int SOUTHWEST_X_IDX = 0;
 	private static final int SOUTHWEST_Y_IDX = 1;
 	private static final int SOUTHEAST_X_IDX = 2;
 	private static final int SOUTHEAST_Y_IDX = 3;
 	private static final int  CALIBRATED_WIDTH_IDX = 4;
 	private static final int  CALIBRATED_HEIGHT_IDX = 5;
 	//private static final int HIGHESTMIDDLE_Y_IDX = 2;
 	private static final int BLOB_COUNT_IDX = 6;
 	private static final int BAD_DATA_COUNTER_THRESHOLD = 10;
 	private static final int POLLING_CYCLE_IN_MSEC = 100;
 	
 	private static final int EXPECTED_ARRAY_SIZE = 7;
 	private static final int EXPECTED_BLOB_COUNT = 2;
 	
 	private static final double CAMERA_FOV_HORIZONTAL_DEGREES = 83.0; // 58.5;
 	
 	// working variables raised to class level to reduce GC pressure inside update task
 	private Dimension _fovDimensions;
 	private Vector _vector;
 	private long _callElapsedTimeMSec;
 	private RawImageData _newTargetRawData;
 	private double _fovXCenterPoint;
 	private double _targetXCenterPoint;
 	private double _fovCenterToTargetXAngleRawDegrees;
 	private int _badDataCounter;
 	
	//============================================================================================
	// constructors follow
	//============================================================================================
    public RoboRealmClient(String kangarooIPv4Addr, int portNo) 
    {        	
    	// create an instance of the RoboRealm API client
    	_rrAPI = new RoboRealmAPI();
    	
    	//Utilities.SimplePingTest(kangarooIPv4Addr);
    	Utilities.RobustPortTest(kangarooIPv4Addr, portNo);
    	
    	// try to connect
        if (!_rrAPI.connect(kangarooIPv4Addr, portNo))
        {
        	DriverStation.reportError("Could not connect to RoboRealm!", false);
        	System.out.println("====> Could not connect to RoboRealm!");
        	
        	_isConnected = false;
        }
        else
        {
        	DriverStation.reportWarning("Connected to RoboRealm!", false);
        	System.out.println("====> Connected to RoboRealm!");
        	
        	_isConnected = true;
        }
        
		// create instance of the task the timer interrupt will execute
		_task = new RoboRealmUpdater();
		
		// create a timer to fire events
		_updaterTimer = new java.util.Timer();
		_updaterTimer.scheduleAtFixedRate(_task, 0, POLLING_CYCLE_IN_MSEC);
		
		//Initialize counter to indicate bad data until proved good
		_badDataCounter = 10;
    }
    
	//============================================================================================
	// Methods follow
	//============================================================================================
    public void TestConnect()
    {
    	Dimension fovDimensions = _rrAPI.getDimension();
    }
    
    // this method changes the active roborealm camera / program
    public void ChangeToCamera(ViSION_CAMERAS visionCamera)
    {
    	_currentVisionCameraName= visionCamera.get_cameraName();
    	
    	if (_rrAPI.setVariable("CamType", _currentVisionCameraName)) {
    		System.out.println("====> RoboRealm Camera switched to " + _currentVisionCameraName);
    	}
    	else {
    		System.out.println("====> FAILED changing RoboRealm Camera to " + _currentVisionCameraName);
    	}
    }
    
    
 	// this method switches the currently running pipeline program
 	public void SwitchProgram(String pipeLineProgramFullPathName)
 	{
 		Boolean isPauseOk = _rrAPI.pause();
 		
 		System.out.println("====> Prepare to Switch Program");
 		
 		if(isPauseOk)
        {
        	//DriverStation.reportError("RoboRealm Program Paused successfully!", false);
        	System.out.println("====> RoboRealm Program Paused succesfully!");
        }
        else
        {
        	//DriverStation.reportError("Error..Could not pause RoboRealm program!", false);
        	System.out.println("====>Error..Could not pause RoboRealm program!");
        }
 		
 		System.out.println("====> Chg Program To: " + pipeLineProgramFullPathName);
 		
 		Boolean isSwitchOk = _rrAPI.loadProgram(pipeLineProgramFullPathName);
 		
 		if(isSwitchOk)
        {
        	//DriverStation.reportError("RoboRealm Program Switched succesfully to: " + pipeLineProgramFullPathName, false);
        	System.out.println("====> RoboRealm Program Switched succesfully to: " + pipeLineProgramFullPathName);
        }
        else
        {
        	//DriverStation.reportError("Error..Could not switch RoboRealm program!", false);
        	System.out.println("====> Error..Could not switch RoboRealm program!");
        }	
 		
 		Boolean isResumeOk = _rrAPI.resume();
 		
 		if(isResumeOk)
        {
        	//DriverStation.reportError("RoboRealm Program Paused succesfully!", false);
        	System.out.println("====> RoboRealm Program Resumed succesfully!");
        }
        else
        {
        	//DriverStation.reportError("Error..Could not pause RoboRealm program!", false);
        	System.out.println("====> Error..Could not resume RoboRealm program!");
        }
 	}
 	
 	
	// update the Dashboard with any Vision specific data values
	public void OutputToSmartDashboard()
	{
		//SmartDashboard.getBoolean("Vision:IsValid", get_isVisionDataValid());
		
		String targetAngle = "?";
		
		//if(get_Angle() == 0)
		//{
		//	targetAngle = String.format("[%.0f RPM] %.0f RPM (%.2f%%) [%.0f%%]", 
		//								-1*_stg1MtrTargetRPM, 
		//								-1*getStg1ActualRPM(), 
		//}
		
		//SmartDashboard.putString("Vision:AdjAngle", get_Angle());
	} 	
	
	//=========================================================================
	//	Task Executed By Timer
	//=========================================================================	
 	
	// poll RoboRealm to read current values
 	public void update() 
 	{ 
 		long startOfCallTimestamp = new Date().getTime();
 		
 		// get the Field Of View Dimensions
 		//_fovDimensions = _rrAPI.getDimension();
 		
 	    // get multiple variables
 		// This must match what is in the config of the "Point Location" pipeline step in RoboRealm
 	    _vector = _rrAPI.getVariables("SW_X,SW_Y,SE_X,SE_Y,SCREEN_WIDTH,SCREEN_HEIGHT,BLOB_COUNT");
 	    _callElapsedTimeMSec = new Date().getTime() - startOfCallTimestamp;
 	    _newTargetRawData = null;
 	    
 	    if (_vector==null)
 	    {
 	    	if(_badDataCounter <= 10 || _badDataCounter % 50 == 0)
 	    	{
 	    		System.out.println("Error in GetVariables, did not return any results [" + _badDataCounter + "]");
 	    	}
 	    	//DriverStation.reportError("Error in GetVariables, did not return any results", false);
 	    	
 	    	//Increment bad data counter
 	    	_badDataCounter += 1;
 	    }
 	    else if(_vector.size() == EXPECTED_ARRAY_SIZE)
 	    {
 	    	_newTargetRawData = new RawImageData();
 	    	
 	    	// parse the results and build the image data
 	    	_newTargetRawData.Timestamp = new Date().getTime();
 	    	_newTargetRawData.SouthWestX = Double.parseDouble((String)_vector.elementAt(SOUTHWEST_X_IDX));
 	    	_newTargetRawData.SouthWestY = Double.parseDouble((String)_vector.elementAt(SOUTHWEST_Y_IDX));
 	    	_newTargetRawData.SouthEastX = Double.parseDouble((String)_vector.elementAt(SOUTHEAST_X_IDX));
 	    	_newTargetRawData.SouthEastY = Double.parseDouble((String)_vector.elementAt(SOUTHEAST_Y_IDX));
 	    	
 	    	_newTargetRawData.BlobCount = Integer.parseInt((String)_vector.elementAt(BLOB_COUNT_IDX));
 	    	
 	    	_fovDimensions = new Dimension();
 	    	_fovDimensions.width = Double.parseDouble((String)_vector.elementAt(CALIBRATED_WIDTH_IDX));
 	    	_fovDimensions.height = Double.parseDouble((String)_vector.elementAt(CALIBRATED_HEIGHT_IDX));
 	    	_newTargetRawData.FOVDimensions = _fovDimensions;
 	    	
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
 	    	_fovXCenterPoint = _fovDimensions.width / 2;
 	    	
 	    	// calc the target center point
 	    	_targetXCenterPoint = Math.round(((_newTargetRawData.SouthEastX + _newTargetRawData.SouthWestX) / 2.0) + 3.0);
 	    	
 	    	_fovCenterToTargetXAngleRawDegrees = ((_fovXCenterPoint - _targetXCenterPoint) * CAMERA_FOV_HORIZONTAL_DEGREES) / _fovDimensions.width;
 	    	// round to 2 decimal places
 	    	_fovCenterToTargetXAngleRawDegrees = Math.round(_fovCenterToTargetXAngleRawDegrees *100) / 100;	
 	    	
    		System.out.println("Angle= " + _fovCenterToTargetXAngleRawDegrees + " mSec=" + _newTargetRawData.ResponseTimeMSec
    				+ " Blob Count= " + _newTargetRawData.BlobCount + " Is on Target= " + Boolean.toString(get_isInGearHangPosition()));
 	
 	    	//Reset the counter 
 	    	_badDataCounter = 0;
 	    }
 	    else
 	    {
 	    	System.out.println("Unexpected Array Size: " + _vector.size());
 	    	_newTargetRawData = null;

 	    	//Increment bad data counter
 	    	_badDataCounter += 1;
 	    }
 	} 
 	
    private class RoboRealmUpdater extends TimerTask 
    { 
 		public void run() 
 		{ 
 			while(true) 
 			{ 
 				update(); 
 				
 				try 
 				{ 
 					// sleep for 10 mSec
 					Thread.sleep(10); 
				} 
 				catch (InterruptedException e) 
 				{ 
 					e.printStackTrace(); 
				} 
 			} 
 		} 
 	}
    
    // =========================================================
    // Property Accessors
    // =========================================================
 	public double get_Angle() 
 	{ 
 		return _fovCenterToTargetXAngleRawDegrees; 
 	}
    
    public double get_fovCenterToTargetXAngleRawDegrees()
    {
    	return _fovCenterToTargetXAngleRawDegrees;
    }
    
    public boolean get_isVisionDataValid()
    {
    	if((_badDataCounter < BAD_DATA_COUNTER_THRESHOLD) && (_newTargetRawData.BlobCount == EXPECTED_BLOB_COUNT))
    	{	
    		return true;
    	}
    	
    	else 
    	{
    		return false;
    	}
    }
    
    public boolean get_isInGearHangPosition()
    {
		if ((_newTargetRawData.SouthEastY > TARGET_MINIMUM_Y_VALUE) && (_newTargetRawData.SouthWestY > TARGET_MINIMUM_Y_VALUE))
		{
			return true;
		}
		else
		{
			return false;
		}
    }
}
