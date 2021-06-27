/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab2.Lab2;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private double x;
  private double y;
  private double theta_angle;
  private double[] currentPosition;
  private double basecolor;
  private boolean passLine =false;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

  }

  /**
   * Here is where the odometer correction code should be run.
   * @author Aleks and Talaal
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    /**
     * Here a base sample is taken to compare for later, to properly judge lines
     */
    Lab2.myColorSample.fetchSample(Lab2.sampleC, 0);
    basecolor = Lab2.sampleC[0];
    while (true) {
      correctionStart = System.currentTimeMillis();
      // TODO Trigger correction (When do I have information to correct?)
      /**
       * Check for a line 
       */
      Lab2.myColorSample.fetchSample(Lab2.sampleC, 0);
		if (((Lab2.sampleC[0] < .20))) {
			Lab2.lineCounter++;
			Sound.beep();
			 /**
		       * Line passed, time to  correct
		       */
			passLine =true;
		}
      // TODO Calculate new (accurate) robot position
		if(passLine) {
			 /**
		       * Obtain current position 
		       */
			currentPosition = odometer.getXYT();
			theta_angle = currentPosition[2];
			y=currentPosition[1];
			x=currentPosition[0];
			 /**
		       * Correct line value based on current distance 
		       */
			if((theta_angle<45 && theta_angle>315)||(theta_angle >135 && theta_angle <225)){
				if(y<20.0) {
					y=0.0;
				}
				else if(y >=15.0 && y <45) {
					y= 30.48;
				}
				else if(y>=45.0 && y<75) {
					y = 30.48*2;
				}
				else {
					
				}
			}
			else {
				if(x<20.0) {
					x=0.0;
				}
				else if(x >=15.0 && x <45) {
					x= 30.48;
				}
				else if(x>=45.0 && x<75) {
					x = 30.48*2;
				}
				else {
					
				}
			}
			 /**
		       * Update XYT 
		       */
			odometer.setXYT(x, y, theta_angle);
			passLine =false;
		}
      // TODO Update odometer with new calculated (and more accurate) vales

      

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
