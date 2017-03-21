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
		//									Position	Desc	Slider	Stg1  Stg2  Is Default?
		//======================================================================================
		
		table.add(new ShooterTableEntry(indexCounter++, " 3ft", .40, -2800, -2600, false));
		table.add(new ShooterTableEntry(indexCounter++, " 4ft", .46, -2867, -2633, false));
		table.add(new ShooterTableEntry(indexCounter++, " 5ft", .51, -2933, -2667, false));
		table.add(new ShooterTableEntry(indexCounter++, " 6ft", .57, -3000, -2700, false));
		table.add(new ShooterTableEntry(indexCounter++, " 7ft", .59, -3067, -2767, false));
		table.add(new ShooterTableEntry(indexCounter++, " 8ft", .60, -3133, -2833, false));
		table.add(new ShooterTableEntry(indexCounter++, " 9ft", .62, -3200, -2900, false));
		table.add(new ShooterTableEntry(indexCounter++, "10ft", .64, -3300, -3000, false));
		table.add(new ShooterTableEntry(indexCounter++, "11ft", .65, -3400, -3100, false));
		table.add(new ShooterTableEntry(indexCounter++, "12ft", .67, -3500, -3200, true));
		table.add(new ShooterTableEntry(indexCounter++, "13ft", .68, -3617, -3317, false));
		table.add(new ShooterTableEntry(indexCounter++, "14ft", .69, -3733, -3433, false));
		table.add(new ShooterTableEntry(indexCounter++, "15ft", .70, -3850, -3550, false));
		
		
		return table;
	}

}
