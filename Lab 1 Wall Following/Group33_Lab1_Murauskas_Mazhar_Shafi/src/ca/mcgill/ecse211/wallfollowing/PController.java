package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 100;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  public static int avg =0;
  private final int MAX_DISTANCE = 100;
  private int filterOut;
  private int constant_test = 5;
  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }
  @Override
  /**
   * ProcessUSData method
   * @param distance 
   * @author Aleks and Talaal
   * 
   * This method processes the error from the desired bandcenter, and proportionally adjusts accordingly using a constant of 5 multiplied by the error, 
   * however the adjustment value is capped at a speed of 200. 
   * @return void
   */
  public void processUSData(int distance) {
    // TODO: process a movement based on the us distance passed in (P style)
	 if(checkDistValidity(distance)) {
	     	this.distance = distance;
	 }
	 int error= bandCenter -this.distance;
     int adjustVal = Math.abs(error*constant_test);
     /**
      * Prevents the adjustment value from being to large
      */
     if (adjustVal > 200) {
    	 adjustVal = 200;
     }
     int left;
     int right;
     /**
      * Robot is within the bandwith and should go straight forward 
      * 
      */
     if( Math.abs(error) <= bandWidth) { //Go forward
     	left = right = MOTOR_SPEED;
     	WallFollowingLab.rightMotor.forward();
     	WallFollowingLab.leftMotor.forward();
     }
     /**
      * Robot is to close to the wall and must turn away, the inside wheel shall turn faster
      * In the case that it is within 20, the robot will turn in place to avoid contact with the wall
      * 
      */
     else if(error >0 ) { // Too close to wall, turn right away from wall
     	if (distance < 20) {
     		WallFollowingLab.rightMotor.forward();
     		WallFollowingLab.leftMotor.backward();
     		right = MOTOR_SPEED;
         	left = MOTOR_SPEED+adjustVal;
 		}
     	else {
     	right = MOTOR_SPEED + adjustVal;
     	left = MOTOR_SPEED;
     	}
     }
     /**
      * 
      * Robot is too far from the wall, and must turn closer so the outside wheel must turn faster 
      */
     else{ //Too far from wall, must turn left toward wall
     	right = MOTOR_SPEED;
     	left = MOTOR_SPEED + adjustVal;
     	WallFollowingLab.rightMotor.forward();
     	WallFollowingLab.leftMotor.forward();
     }
     WallFollowingLab.leftMotor.setSpeed(left);
     WallFollowingLab.rightMotor.setSpeed(right);
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