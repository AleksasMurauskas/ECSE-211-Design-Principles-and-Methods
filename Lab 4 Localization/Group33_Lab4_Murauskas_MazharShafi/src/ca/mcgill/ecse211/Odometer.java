package ca.mcgill.ecse211;
/**
 * Odometer Class
 * Unchanged from Tutorial code
 * Odometer tracks the vehicle's position on an x,y axis as well as it's facing direction in the form of an angle theta
 */


/*
 * File: Odometer.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * Changed to Thread - Jonah Caplan
 * 2015
 * 
 * Class which controls the odometer for the robot
 * 
 * Odometer defines cooridinate system as such...
 * 
 * 					90Deg:pos y-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 180Deg:neg x-axis------------------0Deg:pos x-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 					270Deg:neg y-axis
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 * 
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {

	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final int TIMEOUT_PERIOD = 50;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private double[] oldDH, dDH;

	// constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// default values, modify for your robot
		this.rightRadius = 2.2;
		this.leftRadius = 2.2;
		this.width = 18.15;

		this.x = 0.0;
		this.y = 0.0;
		this.theta = 90.0;
		this.oldDH = new double[2];
		this.dDH = new double[2];
	}
	
	public double[] getXYT() {
		double[] XYT = {x,y,theta};
		return XYT;
	}
	
	public void setTheta(double t) {
		this.theta =t; 
	}
	
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();
		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI
				/ 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}
	public void run() {
		while (true) {
			this.getDisplacementAndHeading(dDH);
			dDH[0] -= oldDH[0];
			dDH[1] -= oldDH[1];

			// update the position in a critical region
			synchronized (this) {
				theta += dDH[1];
				theta = fixDegAngle(theta);

				x += dDH[0] * Math.cos(Math.toRadians(theta));
				y += dDH[0] * Math.sin(Math.toRadians(theta));
			}

			oldDH[0] += dDH[0];
			oldDH[1] += dDH[1];

			try {
				Thread.sleep(TIMEOUT_PERIOD);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			
		}
	}
	public double getX() {
		synchronized (this) {
			return x;
		}
	}
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	// return theta value
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	// set x,y,theta
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	// return x,y,theta
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}

	public EV3LargeRegulatedMotor[] getMotors() {
		return new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor };
	}

	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}

	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	// static 'helper' methods
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}

	public void setY(double newY) {
		y= newY;
		
	}

	public void setX(double newX) {
		x= newX;
		
	}
	public void setXYT(double x2, double y2, double currentTheta) {
		// TODO Auto-generated method stub
		x=x2;
		y=y2;
		theta =currentTheta;
	}
	
}
