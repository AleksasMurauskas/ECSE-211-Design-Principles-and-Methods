package ca.mcgill.ecse211;
/**
 * Main Class
 * @author Aleks Murauskas and Talaal Mazhar Shafi
 * All threads start here
 * This Class has been expanded since lab 3
 * 
 */

import java.io.FileNotFoundException;

import ca.mcgill.ecse211.LCDInfo;
import ca.mcgill.ecse211.Odometer;
import ca.mcgill.ecse211.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Main {
	
	/**
	 * All Ports for motors and sensors are set
	 */
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
	public static Port portL = LocalEV3.get().getPort("S2"); 
	public static SensorModes myColor = new EV3ColorSensor(portL); 
	public static SampleProvider lColorSample = myColor.getMode("Red");      
	/**
	 * Sample Arrays Created
	 */
	public static float[] LData = new float[myColor.sampleSize()];
	public static float[] usData = new float[usSensor.sampleSize()];
	/**
	 * Vehicle information entered 
	 */
	public static final double WHEEL_RAD = 2.2;
	public static final double LENGTH= 16;
	public static final double TRACK = 18.15;
	public static final int D= 25;
	
	/**
	 * Holds other classes
	 */
	private static Navigator nav;
	public static Odometer odometer;
	private static UltrasonicLocalizer usLoc;
	static UltrasonicPoller usPoller;
	

	public static void main(String[] args) throws FileNotFoundException {
		odometer = new Odometer(leftMotor, rightMotor); 
		int choice;
		do {
			/**
			 * LCD Starting Screen Created
			 */
			LCD.clear();
			LCD.drawString("Press Right ", 0, 0);
			LCD.drawString("for Falling Edge", 0, 1);
			LCD.drawString("Press Left", 0, 2);
			LCD.drawString("for Rising Edge", 0, 3);
			choice = Button.waitForAnyPress(); 
			/**
			 * Falling or Rising determined here
			 */
		} while (choice != Button.ID_LEFT && choice != Button.ID_RIGHT);
				if (choice == Button.ID_RIGHT) {
					LCD.clear();
					/**
					 * Ultrasonic sensor used to detect objects in the vehicle's path
					 */
					usPoller = new UltrasonicPoller(usSensor);
					usPoller.start();
					/**
					 * Odometer created to record x and y positions as well as directional theta 
					 */
					odometer.start();
					/**
					 * Thread to update the screen created
					 */
					LCDInfo screen = new LCDInfo(odometer);
					screen.start();
					/**
					 * Ultrasonic Localizer created and rising edge called 
					 */
					usLoc = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, usPoller, D);
					usLoc.fallingEdge();
				}
				else if (choice == Button.ID_LEFT) {
					LCD.clear();
					/**
					 * Ultrasonic sensor used to detect objects in the vehicle's path
					 */
					usPoller = new UltrasonicPoller(usSensor);
					usPoller.start();
					/**
					 * Odometer created to record x and y positions as well as directional theta 
					 */
					odometer.start();
					/**
					 * Thread to update the screen created
					 */
					LCDInfo screen = new LCDInfo(odometer);
					screen.start();	
					/**
					 * Ultrasonic Localizer created and rising edge called 
					 */
					usLoc = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, usPoller, D);
					usLoc.risingEdge();
				}
			/**
			 * Turn to the calculated 90 degree angle 
			 */
			 turnRight(odometer.getXYT()[2]-90);
			 Button.waitForAnyPress();
			 /**
			  * Continues to Light Localization
			  */
			 LightLocalizer LL = new LightLocalizer(leftMotor,rightMotor,lColorSample ,LData, odometer);
			 /**
			  * Preparitive step occurs here, finds starting x and y for spin
			  */
			 LL.Preparation();
			 
			 LL.localize();
			 /**
		      * Navigation thread created
			  */
			 nav = new Navigator(odometer, usPoller);
			 nav.start();
			 /**
			  * Navigates to the origin
			  */
			 nav.travelTo(0, 0);
			 try {
		          Thread.sleep(10);
		        } catch (InterruptedException e) {
		          e.printStackTrace();
		        }
			 nav.turnTo(90, true);
			 /**
			  * Completion
			  */
		while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
		}
		System.exit(0);
	}
	/**
	 * The robot turns a certain desired degree to the robot's right
	 * @param deg
	 */
	 static void turnRight(double deg) {
		    leftMotor.setSpeed(50);
		    rightMotor.setSpeed(50);
		    
		    leftMotor.rotate(+convertAngle(WHEEL_RAD, Main.TRACK, deg), true);
		    rightMotor.rotate(-convertAngle(WHEEL_RAD, Main.TRACK, deg), false);
		  }
	 /**
	  *  Computes the proper amount of rotations for the robot to travel a desired distance
	  * @param radius
	  * @param distance
	  * @return converted distance
	  */
	 private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }
	 /**
	  * Computes the amount of rotations each wheel would need to perform a rotation of the 
	  * desired distance
	  * @param radius
	  * @param width
	  * @param angle
	  * @return converted angle
	  */
	 private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
		  }
	 
	
}