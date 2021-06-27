package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  private static final int FILTER_OUT = 20;
  private int filterControl;
  private final int MAX_DISTANCE = 100;
  private int filterOut;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();  
  }
  /**
   * ProcessUSData method
   * @param distance 
   * @author Aleks and Talaal
   * 
   * This method will take the input from the ultrasonic sensor, compute the error from the bandwidth, then bang left or right for the necessary turn.
   * 
   * @return void 
   */
  @Override
  public void processUSData(int distance) {
    
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    if(checkDistValidity(distance)) {
    	this.distance = distance;
    }
    int error = bandCenter -this.distance;
    int highspeed = motorHigh;
    int lowspeed = motorLow;
    int left;
    int right;
    /**
     * Robot is within the bandwith and should go straight forward 
     * 
     */
    if( Math.abs(error) <= bandwidth) { //Go forward
    	left = right = motorLow;
    	WallFollowingLab.rightMotor.forward();
    	WallFollowingLab.leftMotor.forward();
    }
    /**
     * Robot is to close to the wall and must turn away, the inside wheel shall turn faster
     * In the case that it is within 18, the robot will turn in place to avoid contact with the wall
     * 
     */
    else if(error >0) { // Too close to wall, turn right away from wall
    	if (distance < 18) {
			  WallFollowingLab.leftMotor.backward();
			  WallFollowingLab.rightMotor.forward();
			  left = lowspeed;
			  right = highspeed+200;
		  }
    	else {
    	left = highspeed;
    	right = lowspeed;
    	WallFollowingLab.rightMotor.forward();
    	WallFollowingLab.leftMotor.forward();
    	 }
    }
    /**
     * 
     * Robot is too far from the wall, and must turn closer so the outside wheel must turn faster 
     */
    else{ 
    	if(distance > (bandCenter + bandwidth)+20) {
    		highspeed += 50;
    	}
    	left = lowspeed;
    	right = highspeed-40;
    	WallFollowingLab.rightMotor.forward();
    	WallFollowingLab.leftMotor.forward();
    }
    WallFollowingLab.leftMotor.setSpeed(left);
    WallFollowingLab.leftMotor.setSpeed(right);
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
  /**
   * This method implements the rudimentary filter
   * 
   */
  public boolean checkDistValidity(int distance) {
	  if (distance == Integer.MAX_VALUE) {
		  return false;
	  }
	// rudimentary filter - toss out invalid samples corresponding to null
	    // signal.
	    // (n.b. this was not included in the Bang-bang controller, but easily
	    // could have).
	  if (distance >= MAX_DISTANCE && filterControl < filterOut) {
	      // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	      return false;  
	    } 
		else if (distance >= MAX_DISTANCE) {
	      // We have repeated large values, so there must actually be nothing
	      // there: leave the distance alone
	      return true;   
	    } 
	    else {
	      // distance went below 255: reset filter and leave
	      // distance alone.
	      filterControl = 0;
	      return true;
	    }
  }
}
