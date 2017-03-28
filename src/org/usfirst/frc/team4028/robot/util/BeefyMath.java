package org.usfirst.frc.team4028.robot.util;

/* 
 * Holds the beefiest, and therefore the best, methods pertaining
 * to mathematical computations used on the robot. Partially inspired
 * by Beefulus
 * 
 *          (__)
 *        	(oo)                      
 *	 /-------\/                   
 *  / |     ||                                  
 *    ||----||
 *    ^^    ^^
 */

public class BeefyMath {
  public static double arctan(double heading) {
	  double degrees = Math.toDegrees(Math.atan(heading));
	  return degrees;
  }
}