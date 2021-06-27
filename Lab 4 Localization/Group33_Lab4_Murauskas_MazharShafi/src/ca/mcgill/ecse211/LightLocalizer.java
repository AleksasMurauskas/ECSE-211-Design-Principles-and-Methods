package ca.mcgill.ecse211;


import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
/**
 * 
 * @author Aleks and Talaal
 * Class: Light Localizer
 * Localizes the location of the robot using two phases
 * 1. A prepratory step to move the robot into position and correct the odometer
 * 2. Complete a circular rotation and use the crossing lines to find the true position of the robot 
 */
public class LightLocalizer {
  
	private static float[] lData;
	private static SampleProvider lSampler;
	private static double xcrossA;
	private static double xcrossB;
	private static double ycrossA;
	private static double ycrossB;
	private static final double LENGTH = Main.LENGTH;
  
	static EV3LargeRegulatedMotor leftMotor;
	static EV3LargeRegulatedMotor rightMotor;
	static final int ROTATESPEED = 50;
	private static Odometer odo;
  
	public LightLocalizer(EV3LargeRegulatedMotor left, EV3LargeRegulatedMotor right,SampleProvider samp,float[] l, Odometer od) {
		leftMotor =left;
		rightMotor = right;
		lData =l;
		lSampler = samp;
		odo =od;
	}
	/**Preparation method
	 * The Robot prepares to complete it's localization cycle
	 * First the robot centers itself on a desired  point by passing a line along the X axis to correct the odometer
	 * and reverses to a desired point and repeats for the y axis. Then Travels to a desired point to begin localization
	 */
	public void Preparation() {
		leftMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		leftMotor.forward();
		rightMotor.forward();
		boolean passedLine = false;
		while (!passedLine) {
			lSampler.fetchSample(lData, 0);
			if (100 * lData[0] < 40) {
				passedLine = true;
			}
		}
		/**
		 * Y=0 line passed, odometer corrected
		 */
		odo.setY(LENGTH);
		travelStraight((int) -(LENGTH + 10));
		turnRight(90);
		leftMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		leftMotor.forward();
		rightMotor.forward();
		passedLine = false;
		/**
		 * X=0 line passed, odometer corrected
		 */
		while (!passedLine) {
			lSampler.fetchSample(lData, 0);
			if (100 * lData[0] < 40) {
				passedLine = true;
			}
		}
		odo.setX(LENGTH);
		travelStraight((int) -(LENGTH + 10));
		turnLeft(90);
		/**
		 * Navigator created to move to the desired point
		 */
		Navigator nav = new Navigator(odo, Main.usPoller);
		nav.start();
		nav.travelTo(-6, -6);
		/**
		 * Completion
		 */
		while (nav.isTravelling()) {
		} 
		nav.notDone = false;
	}
  
	
	/**Complete a rotation of 360 degrees, recording the angle at which each line is passed at.
	 * Those angles are then inverted and used in a sequence of equation to localize the robot. 
	 * 
	 */
	public void localize() {
		double initialAng = odo.getXYT()[2];
		boolean onBlackLine = false;
		/**
		 * Turn until first line is found
		 */
		while (!onBlackLine) {
			lSampler.fetchSample(lData, 0);
			if (100 * lData[0] < 40) {
				onBlackLine = true;
			}
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);
        leftMotor.forward();
        rightMotor.backward();
        }
		Sound.beep();
		/**
		 * Record the angle of crossing
		 */
		xcrossA = Main.odometer.getAng();
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);
		leftMotor.forward();
		rightMotor.backward();
		try {
			Thread.sleep(400);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		onBlackLine = false;
		/**
		 * Turn until Second line is found
		 */
		while (!onBlackLine) {
			lSampler.fetchSample(lData, 0);
			if (100 * lData[0] < 40) {
				onBlackLine = true;
			}
			leftMotor.setSpeed(ROTATESPEED);
			rightMotor.setSpeed(ROTATESPEED);
			leftMotor.forward();
			rightMotor.backward();
		}
		/**
		 * Record the angle of crossing
		 */
		ycrossA = odo.getAng();
		try {
			Thread.sleep(400);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		onBlackLine = false;
		/**
		 * Turn until third line is found
		 */
		while (!onBlackLine) {
			lSampler.fetchSample(lData, 0);
			if (100 * lData[0] < 40) {
				onBlackLine = true;
			}
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);
        leftMotor.forward();
        rightMotor.backward();
		}
		/**
		 * Record the angle of crossing
		 */
		xcrossB = odo.getAng();
		try {
			Thread.sleep(400);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		onBlackLine = false;
		/**
		 * Turn until final line is found
		 */
		while (!onBlackLine) {

			lSampler.fetchSample(lData, 0);
			if (100 * lData[0] < 40) {
				onBlackLine = true;
			}
			leftMotor.setSpeed(ROTATESPEED);
			rightMotor.setSpeed(ROTATESPEED);
			leftMotor.forward();
			rightMotor.backward();
		}
		/**
		 * Record the angle of crossing
		 */
		ycrossB = odo.getAng();
        while (Main.odometer.getAng() > initialAng) {
        	try {
        		Thread.sleep(10);
        	} catch (InterruptedException e) {
        		e.printStackTrace();
        	}
        }
        /**
		 * Completed Circle
		 */
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        /**
		 * Invert all found angles for calculation 
		 */
        xcrossA = invert(xcrossA);
        xcrossB = invert(xcrossB);
        ycrossA = invert(ycrossA);
        ycrossB = invert(ycrossB);
        /**
         * Calculate Theta along the x axis
         */
        double X = Math.abs(xcrossA - xcrossB);
        if (X > 180) {
        	X = 360 - X;
        }
        /**
         * Calculate Theta along the Y axis
         */
        double Y= Math.abs(ycrossA - ycrossB);
        if (Y > 180) {
        	Y = 360 - Y;
        }
        /**
         * Find localized position
         */
        double endx =  Math.cos(Math.toRadians(Y) / 2)*-LENGTH;
        double endy =  Math.cos(Math.toRadians(X) / 2)*-LENGTH;
        double currentTheta = odo.getAng();
        /**
         * Set Localized position
         */
        odo.setXYT(endx, endy, currentTheta); 
	}
	
	/**
	 * Invert an angle to be used in localization equations
	 * @param angle
	 * @return
	 */
	private static double invert(double angle) {
	  return (angle + 180) % 360;
	}
	//public static double convert(double ang) {
	//	return Odometer.fixDegAngle(((-ang + 90) % 360));
	//}
	/**
	 * Robot Travels straight a desired distance
	 * @param dist
	 */
	static void travelStraight(int dist) {
	    leftMotor.setSpeed(100);
	    rightMotor.setSpeed(100);
	    leftMotor.rotate(convertDistance(Main.WHEEL_RAD, dist), true);
	    rightMotor.rotate(convertDistance(Main.WHEEL_RAD, dist), false);

	  }
  /**
   * The robot turns to left the desired number of degrees
   * @param deg
   */
  static void turnLeft(double deg) {
	    leftMotor.setSpeed(50);
	    rightMotor.setSpeed(50);
	    leftMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, deg), true);
	    rightMotor.rotate(+convertAngle(Main.WHEEL_RAD, Main.TRACK, deg), false);
	  }
  /**
	 * The robot turns a certain desired degree to the robot's right
	 * @param deg
	 */
	 static void turnRight(double deg) {
		    leftMotor.setSpeed(50);
		    rightMotor.setSpeed(50);
		    
		    leftMotor.rotate(+convertAngle(Main.WHEEL_RAD, Main.TRACK, deg), true);
		    rightMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, deg), false);
		  }
  
	 private static int convertDistance(double radius, double distance) {
		 return (int) ((180.0 * distance) / (Math.PI * radius));
	  	}
	 private static int convertAngle(double radius, double width, double angle) {
		 return convertDistance(radius, Math.PI * width * angle / 360.0);
	 }
}
