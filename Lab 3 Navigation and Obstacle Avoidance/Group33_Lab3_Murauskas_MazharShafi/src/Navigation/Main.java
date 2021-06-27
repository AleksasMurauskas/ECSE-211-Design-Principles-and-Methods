package Navigation;
/**
 * Main Class
 * @author Aleks Murauskas and Talaal Mazhar Shafi
 * All threads start here
 * 
 * 
 */

import java.io.FileNotFoundException;

import Display.LCDInfo;
import Display.Log;
import Odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class Main {
	
	/**
	 * All Ports for motors and sensors are set
	 */
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
	/**
	 * Vehicle information entered 
	 */
	public static final double WHEEL_RADIUS = 2.2;
	public static final double TRACK = 18.15;
	private static Navigator nav;
	private static Odometer odometer;
	
	public static void main(String[] args) throws FileNotFoundException {
		/**
		 * Log of Vehicle states recorded into a log file
		 */
		Log.setLogging(true,true,false,true);
		//Uncomment this line to print to a file
		Log.setLogWriter(System.currentTimeMillis() + ".log");
		/**
		 * Odometer created to record x and y positions as well as directional theta 
		 */
		odometer = new Odometer(leftMotor, rightMotor);
		odometer.start();
		/**
		 * Ultrasonic sensor used to detect objects in the vehicle's path
		 */
		UltrasonicPoller usPoller = new UltrasonicPoller(usSensor);
		usPoller.start();
		/**
		 * Navigation thread created
		 */
		nav = new Navigator(odometer,usPoller);
		nav.start();
		/**
		 * Thread to update the screen created
		 */
		LCDInfo screen = new LCDInfo(odometer);
		screen.start();
		/**
		 * Course begins
		 */
		while (Button.waitForAnyPress() != Button.ID_ENTER){
		}
		completeCourse();
		while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
		}
		System.exit(0);
	}

	private static void completeCourse() {
		/**
		 * All maps stored here for easy use
		 */
		int[][] mapDemo = {{60,30},{30,30},{30,60},{60,0}};
		int[][] map1 = {{0,60},{30,30},{60,60},{60,30},{30,0}};
		int[][] map2 = {{30,30},{0,61},{61,61},{61,30},{30,0}};
		int[][] map3 = {{30,0},{61,30},{61,61},{0,61},{30,30}};
		int[][] map4 = {{0,30},{30,61},{30,0},{61,30},{61,61}};
		/**
		 * Map waypoints are played through  here
		 */
		for(int[] point : mapDemo){
			nav.travelTo(point[0],point[1],true);
			while(nav.isTravelling()){
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
	
}