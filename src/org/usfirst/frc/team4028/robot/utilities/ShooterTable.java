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
		table.add(new ShooterTableEntry(indexCounter++, "Pos A", .64, -3300, -3000, false));
		table.add(new ShooterTableEntry(indexCounter++, "Pos B", .67, -3500, -3200, true));
		table.add(new ShooterTableEntry(indexCounter++, "Pos C", .70, -3700, -3400, false));
		//table.add(new ShooterTableEntry(indexCounter++, "Pos C",.65, -3600, -3750, true));
		//table.add(new ShooterTableEntry(indexCounter++, "Pos D",.70, -3650, -3800, false));
		
		return table;
	}

}
