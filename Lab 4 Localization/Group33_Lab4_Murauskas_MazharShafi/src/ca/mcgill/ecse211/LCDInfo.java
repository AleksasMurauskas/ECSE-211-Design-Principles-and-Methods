package ca.mcgill.ecse211;
import ca.mcgill.ecse211.Odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
/**
 * LCDInfo class
 * Updates the screen to display odometer values
 */
public class LCDInfo extends Thread{
	/**
	 * Sets class fields
	 */
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();;
	
	// arrays for displaying data
	private double [] pos;
	
	public LCDInfo(Odometer odo) {
		this.odo = odo;
		pos = new double [3];
		
	}
	public void run() { 
		/**
		 * Infinite loop to continuously update the screen
		 */
		while(true) {
		
		odo.getPosition(pos);
		LCD.clear();
		/**
		 * Displays X,Y, and Theta
		 */
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("T: ", 0, 2);
		LCD.drawInt((int)(pos[0] * 10), 3, 0);
		LCD.drawInt((int)(pos[1] * 10), 3, 1);
		LCD.drawInt((int)pos[2], 3, 2);
		LCD.drawString("Push Enter to Start", 0, 4);
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		}
	}
}
