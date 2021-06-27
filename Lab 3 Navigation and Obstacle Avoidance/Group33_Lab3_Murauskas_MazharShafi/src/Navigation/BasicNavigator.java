package Navigation;
import Odometer.Odometer;
/*
 * File: Navigation.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * Helper methods and extend Thread - Jonah Caplan
 * 2015
 * 
 * Movement control class (turnTo, travelTo, flt, localize)
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * BasicNavigator Class
 * Obtained from tutorial code 
 */
public class BasicNavigator extends Thread{
	/**
	 * Sets Navigation Variables
	 */
	final static int FAST = 200, SLOW = 100, ACCELERATION = 4000;
	final static double DEG_ERR = 1, CM_ERR = 1;
	Odometer odometer;
	EV3LargeRegulatedMotor leftMotor, rightMotor;

	public BasicNavigator(Odometer odo) {
		this.odometer = odo;
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}
	/**
	 * setSpeeds method sets the speeds of both motors and sets them in reverse if the value is negative
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}
	/**
	 * setSpeeds method sets the speeds of both motors and sets them in reverse if the value is negative
	 */
	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}
	/**
	 * setFloat sets both motors to enter float mode
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}
	/**
	 * travelTo is  a method that drives the robot to the inputed x and y axis, continues to do this until done
	 * @param x position
	 * @param y position 
	 */
	public void travelTo(double x, double y) {
		double minAng;
		while (!checkIfDone(x,y)) {
			minAng = getDestAngle(x,y);
			this.turnTo(minAng, false);
			this.setSpeeds(FAST, FAST);
		}
		this.setSpeeds(0, 0);
	}
	/**
	 * checkIfDone inspects if the robot has reached it's destination, within the specified error
	 * @param x position
	 * @param y position 
	 * @return true if the robot has reached the desired position
	 */
	protected boolean checkIfDone(double x, double y) {
		return Math.abs(x - odometer.getX()) < CM_ERR
				&& Math.abs(y - odometer.getY()) < CM_ERR;
	}
	/**
	 * facingDest inspects if the robot is facing the correct angle, within the specified error
	 * @param desired angle
	 * @return true if the robot is facing the correct direction
	 */
	protected boolean facingDest(double angle) {
		return Math.abs(angle - odometer.getAng()) < DEG_ERR;
	}
	/**
	 * getDestAngle computes the necessary angle the robot must follow to get to the waypoint, and then finds a minimal angle direction
	 * @param x position
	 * @param y position 
	 * @return the minimal angle
	 * */
	protected double getDestAngle(double x, double y) {
		double minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX()))
				* (180.0 / Math.PI);
		if (minAng < 0) {
			minAng += 360.0;
		}
		return minAng;
	}
	/**
	 * turnTo rotates the robot to a desired angle
	 * @param desired angle to turn to 
	 * @param stop boolean
	 * 
	 * */
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-SLOW, SLOW);
			} else if (error < 0.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else if (error > 180.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else {
				this.setSpeeds(-SLOW, SLOW);
			}
		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}
	/**
	 * goForward orders the motors to go straight forward a specified distance 
	 * @param distance to be traveled
	 *  
	 * */
	public void goForward(double distance) {
		this.travelTo(
				odometer.getX()
						+ Math.cos(Math.toRadians(this.odometer.getAng()))
						* distance,
				odometer.getY()
						+ Math.sin(Math.toRadians(this.odometer.getAng()))
						* distance);
	}
}
