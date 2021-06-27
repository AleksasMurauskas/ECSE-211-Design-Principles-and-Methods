package ca.mcgill.ecse211;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * class: UltrasonicLocalizer
 * The robot used the ultrasonic sensor to detect two walls, and uses the 
 * 
 */
public class UltrasonicLocalizer {
  
	static final int ROTATESPEED= 40; 
	private double thetaA;
  	private double thetaB;
  	private double thetaC;
  	private double thetaD;
	private double alphaAngle;
	private double betaAngle;
  	private int k = 3; 
  	static int D;
  	double currentD =0;
  	double deltaT;
  	
  	static Odometer odo;
	static UltrasonicPoller usPoller;
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
 
	/**
	 * Constructor
	 * @param left
	 * @param right
	 * @param od
	 * @param poll
	 * @param d
	 */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor left,EV3LargeRegulatedMotor right, Odometer od, UltrasonicPoller poll, int d ) {
  		odo = od;
  		usPoller=poll;
  		leftMotor =left;
  		rightMotor =right;
  		D=d;
	}
	/**
	 * the robot continuously rotate to the left
	 */
	  private void RotateLeft() {
	    // TODO Auto-generated method stub
	    leftMotor.setSpeed(ROTATESPEED);
	    rightMotor.setSpeed(ROTATESPEED);
	    leftMotor.backward();
	    rightMotor.forward();
	  }
	  /**
	   * the robot continuously rotate to the right
	   */
	  private void RotateRight() {
	    // TODO Auto-generated method stub
	    leftMotor.setSpeed(ROTATESPEED);
	    rightMotor.setSpeed(ROTATESPEED);
	    leftMotor.forward();
	    rightMotor.backward();
	  }
	  /**
	   * Returns a Ultrasonic poller value, after it is filtered for distances greater than 50 
	   * @return distance
	   */
  	private int getFilteredUSData() {
  		int distance = 0;
  		distance =  usPoller.getDistance();
  		if (distance>50) {
  			distance = 50;
  		}
  		return distance;
  	}
  /**
   * The robot rotates to localize, using falling edges to find thetas
   * The falling edge will begin at the top of the buffer and conclude at the bottom of it
   */
  	public void fallingEdge() {
  		/**
  		 * Rotate Right until Within Buffer range
  		 */
  		while(getFilteredUSData()>D+k) {
  			RotateRight();
  		} 
  		/**
  		 * Set first key angle
  		 */
  		thetaA= odo.getXYT()[2];
  		/**
  		 * Rotate Right until out of Buffer range
  		 */
  		while(getFilteredUSData()>D-k) {
  			RotateRight();
  		} 
  		/**
  		 * Set second key angle
  		 */
  		thetaB = odo.getXYT()[2];
  		/**
  		 * Begin turning to the left
  		 */
  		RotateLeft();
  		try {
  			Thread.sleep(1500); 
  		} 	catch (InterruptedException e) {
  			e.printStackTrace();
  		}
  		/**
  		 * Rotate left until The top of the buffer is reached
  		 */
  		while(getFilteredUSData()>D+k) {
  			RotateLeft();
  		}
  		/**
  		 * set third key angle
  		 */
  		thetaC =odo.getXYT()[2];
  		/**
  		 * Rotate left until The bottom of the buffer is reached
  		 */
  		while(getFilteredUSData()>D-k) {
  			RotateLeft();
    }
  	/**
  	 * Set final theta and stop the robot
  	 */
    thetaD = odo.getXYT()[2];
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    /**
     * Find the averages between the two angles found on the edge of each buffer
     */
    alphaAngle= (thetaA+thetaB)/2;
    betaAngle= (thetaC+thetaD)/2;
    double deltaT;
    /**
     * Calculate the delta T
     */
    if (alphaAngle<betaAngle) {
      deltaT= (45-((alphaAngle+betaAngle)/2))%360;
    } else {
      deltaT= (225-((alphaAngle+betaAngle)/2))%360;
    }
    double t = odo.getXYT()[2];
    /**
     * Set current theta
     */
    odo.setTheta(t+deltaT);
    /**
     * Completion
     */
  }

  	 /**
     * The robot rotates to localize, using falling edges to find thetas
     * The rising edge will begin at the bottom of the buffer and conclude at the top of it
     */
  public void risingEdge() {  
	  /**
		 * Rotate Right until larger than Buffer Floor
		 */
    while(getFilteredUSData()<D-k) {
    	RotateRight();
    } 
    /**
		 * Set first key angle
		 */
    thetaA= odo.getXYT()[2];
    /**
	 * Rotate Right until larger than Buffer Ceiling
	 */
    while(getFilteredUSData()<D+k) {
    	RotateRight();
      } 
    /**
  	 * Set second theta and begin turning left
  	 */
    thetaB = odo.getXYT()[2];
    RotateLeft();
    try {
      Thread.sleep(1500); 
  } catch (InterruptedException e) {
      e.printStackTrace();
  }
    /**
	 * Rotate Left until larger than Buffer FLoor
	 */
    while(getFilteredUSData()<D-k) {
    	RotateLeft();
    }
    /**
     * Set third key angle
     */
    thetaC = odo.getXYT()[2];
    /**
   	 * Rotate Left until larger than Buffer Ceil
   	 */
    while(getFilteredUSData()<D+k) {
      leftMotor.setSpeed(ROTATESPEED);
      rightMotor.setSpeed(ROTATESPEED);
      leftMotor.backward();
      rightMotor.forward();
    }
    /**
  	 * Set final theta and stop the robot
  	 */
    thetaD = odo.getXYT()[2];
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    /**
     * Find the averages between the two angles found on the edge of each buffer
     */
    alphaAngle= (thetaA+thetaB)/2;
    betaAngle= (thetaC+thetaD)/2;
    double deltaT;
    /**
     * Calculate the delta T
     */
    if (alphaAngle<betaAngle) {
      deltaT= (225-((alphaAngle+betaAngle)/2))%360;
    } else {
      deltaT= (45-((alphaAngle+betaAngle)/2))%360;
    }
    double t = odo.getXYT()[2];
    /**
     * Set current theta
     */
    odo.setTheta(t+deltaT);
    /**
     * Completion
     */
  } 
}
