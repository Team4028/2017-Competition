package org.usfirst.frc.team4028.robot.utilities;

import java.util.Iterator;
import java.util.LinkedList;

//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	====================================
//	1		TomB		29.Mar.2017	Implemented separate table for Auton 
//------------------------------------------------------

public class ShooterTable {

	// define class level working variables
	private LinkedList<ShooterTableEntry> _telopTable = null;
	private LinkedList<ShooterTableEntry> _autonTable = null;
	private int _currentIndex = 0;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public ShooterTable(){
		_telopTable = LoadTable();
		_autonTable = LoadAutonTable();
		
		// set the initial value of the current index to the 1st one marked as default
		_currentIndex = 1;
		ShooterTableEntry ste;
		Iterator<ShooterTableEntry> itr = _telopTable.iterator();
		while(itr.hasNext()) {
			ste = itr.next();
			if (ste.IsDefault == true) {
				_currentIndex = ste.Index;
				break;
			}
		}	
	}

	//============================================================================================
	// methods follow
	//============================================================================================
	public ShooterTableEntry getNextEntry()
	{
		if(!get_IsAtUpperEntry()) {
			_currentIndex++;
		}
		
		return _telopTable.get(_currentIndex);
	}
	
	public ShooterTableEntry getCurrentEntry()
	{
		return _telopTable.get(_currentIndex);
	}
	
	public ShooterTableEntry getPreviousEntry()
	{
		if(!get_IsAtLowerEntry()) {
			_currentIndex--;
		}

		return _telopTable.get(_currentIndex);
	}
	
	public ShooterTableEntry getAutonEntryForDistance(int targetDistanceInInches)
	{
		return getEntryForDistance(targetDistanceInInches, _telopTable);
	}
	
	public ShooterTableEntry getTelopEntryForDistance(int targetDistanceInInches)
	{
		return getEntryForDistance(targetDistanceInInches, _autonTable);
	}
	
	// Return the correct SHooterTableEntry for a given distance
	// ShooterTableEntry ste = _shooterTable.getEntryForDistance(130);
	// System.out.println("..STE Entry: " + ste.Description);
	private ShooterTableEntry getEntryForDistance(int targetDistanceInInches, LinkedList<ShooterTableEntry> table)
	{
		// set the initial value of the current index to the 1st one marked as default
		int currentIndex = 1;
		ShooterTableEntry selectedSte = null;
		ShooterTableEntry previousSte = null;
		ShooterTableEntry currentSte = null;
		
		// loop thru and try to find the entries above and below the target distance
		Iterator<ShooterTableEntry> itr = table.iterator();
		while(itr.hasNext()) {
			currentSte = itr.next();
			
			// if target distance is longer, continue looping
			if (currentSte.DistanceInInches < targetDistanceInInches) {
				previousSte = currentSte;
				continue;
			}
			// else stop looping
			else if (currentSte.DistanceInInches >= targetDistanceInInches) {
				break;
			}
		}	
		
		// now decide which one to use
		if (currentSte == null) {
			selectedSte = table.get(1);	// should never happen!(just default to 1st entry
		}
		else if((previousSte == null) || (currentSte == previousSte)){
			selectedSte = currentSte;
		}
		else {
			// we have 2 options, need to decide which one is closer
			int previousSteDeltaDistance = Math.abs(targetDistanceInInches - previousSte.DistanceInInches);
			int thisSteDeltaDistance = Math.abs(currentSte.DistanceInInches - targetDistanceInInches);
			
			if(thisSteDeltaDistance <= previousSteDeltaDistance) {
				selectedSte = currentSte;
			}
			else {
				selectedSte = previousSte;
			}
		}
		
		return selectedSte;
	}
	
	//============================================================================================
	// properties follow
	//============================================================================================
	
	public Boolean get_IsAtUpperEntry()
	{
		if (_currentIndex == _telopTable.size() - 1){
			return true;
		}
		else {
			return false;
		}
	}
	
	public Boolean get_IsAtLowerEntry()
	{
		if (_currentIndex == 0){
			return true;
		}
		else {
			return false;
		}
	}
	
	//============================================================================================
	// helpers follow
	//============================================================================================
	// create a linked list
	private LinkedList<ShooterTableEntry> LoadTable() {

		LinkedList<ShooterTableEntry> table = new LinkedList<ShooterTableEntry>();
		
		int indexCounter = 0;
		//======================================================================================
		//									Position	Inches Desc	Slider	Stg1  Stg2  Is Default?
		//======================================================================================
		
		table.add(new ShooterTableEntry(indexCounter++, 36, " 3ft", .40, -2800, -2600, false));
		table.add(new ShooterTableEntry(indexCounter++, 48, " 4ft", .46, -2867, -2633, false));
		table.add(new ShooterTableEntry(indexCounter++, 60, " 5ft", .51, -2933, -2667, false));
		table.add(new ShooterTableEntry(indexCounter++, 72, " 6ft", .57, -3000, -2700, false));
		table.add(new ShooterTableEntry(indexCounter++, 84, " 7ft", .59, -3067, -2767, false));
		table.add(new ShooterTableEntry(indexCounter++, 96, " 8ft", .60, -3133, -2833, false));
		table.add(new ShooterTableEntry(indexCounter++, 108, " 9ft", .62, -3200, -2900, false));
		table.add(new ShooterTableEntry(indexCounter++, 120, "10ft", .64, -3300, -3000, false));
		table.add(new ShooterTableEntry(indexCounter++, 132, "11ft", .65, -3400, -3100, false));
		table.add(new ShooterTableEntry(indexCounter++, 144, "12ft", .67, -3500, -3200, true));
		table.add(new ShooterTableEntry(indexCounter++, 156, "13ft", .68, -3617, -3317, false));
		table.add(new ShooterTableEntry(indexCounter++, 168, "14ft", .69, -3733, -3433, false));
		table.add(new ShooterTableEntry(indexCounter++, 180, "15ft", .70, -3850, -3550, false));
		
		
		return table;
	}

	private LinkedList<ShooterTableEntry> LoadAutonTable() {

		LinkedList<ShooterTableEntry> table = new LinkedList<ShooterTableEntry>();
		
		int indexCounter = 0;
		//======================================================================================
		//									Position	Inches Desc	Slider	Stg1  Stg2  Is Default?
		//======================================================================================

		table.add(new ShooterTableEntry(indexCounter++, 172, "14' 4ft", .69, -3771, -3471, false));
		table.add(new ShooterTableEntry(indexCounter++, 175, "14' 7ft", .69, -3811, -3511, false));
		
		return table;
	}
	
}
