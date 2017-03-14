package org.usfirst.frc.team4028.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;

public class LEDRingRelay {

	DigitalOutput _LEDDIO;
	
	//=========================================================================
	//	Constructor(s)
	//=========================================================================
	
	public LEDRingRelay(int channel) {
		_LEDDIO = new DigitalOutput(channel);
	}
	
	//=========================================================================
	//	Methods
	//=========================================================================

	public void turnOffLEDs(){
		_LEDDIO.set(false);		
	}

	public void turnOnLEDs(){
		_LEDDIO.set(true);
	}
}

