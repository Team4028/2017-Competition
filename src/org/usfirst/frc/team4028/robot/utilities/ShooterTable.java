package org.usfirst.frc.team4028.robot.utilities;

import java.util.Iterator;
import java.util.LinkedList;

public class ShooterTable {

	// define class level working variables
	private LinkedList<ShooterTableEntry> _table = null;
	private int _currentIndex = 0;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public ShooterTable(){
		_table = LoadTable();
		
		// set the initial value of the current index to the 1st one marked as default
		_currentIndex = 1;
		ShooterTableEntry ste;
		Iterator<ShooterTableEntry> itr = _table.iterator();
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
		
		return _table.get(_currentIndex);
	}
	
	public ShooterTableEntry getCurrentEntry()
	{
		return _table.get(_currentIndex);
	}
	
	public ShooterTableEntry getPreviousEntry()
	{
		if(!get_IsAtLowerEntry()) {
			_currentIndex--;
		}

		return _table.get(_currentIndex);
	}
	
	// Return the correct SHooterTableEntry for a given distance
	// ShooterTableEntry ste = _shooterTable.getEntryForDistance(130);
	// System.out.println("..STE Entry: " + ste.Description);
	public ShooterTableEntry getEntryForDistance(int targetDistanceInInches)
	{
		// set the initial value of the current index to the 1st one marked as default
		int currentIndex = 1;
		ShooterTableEntry selectedSte = null;
		ShooterTableEntry previousSte = null;
		ShooterTableEntry currentSte = null;
		
		// loop thru and try to find the entries above and below the target distance
		Iterator<ShooterTableEntry> itr = _table.iterator();
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
			selectedSte = _table.get(1);	// should never happen!(just default to 1st entry
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
		if (_currentIndex == _table.size() - 1){
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

}
