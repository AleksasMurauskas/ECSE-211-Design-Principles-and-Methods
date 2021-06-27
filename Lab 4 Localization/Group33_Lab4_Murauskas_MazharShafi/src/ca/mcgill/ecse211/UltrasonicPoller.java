package ca.mcgill.ecse211;
/**
 * Ultrasonic Poller Class
 * Unchanged from Tutorial Code
 * Polls the ultrasonic sensor for a possible obstacle in the path of the object
 */

import lejos.robotics.SampleProvider;

public class UltrasonicPoller extends Thread {
	private SampleProvider us;
	private float[] usData;
	int distance;

	public UltrasonicPoller(SampleProvider us) {
		this.us = us;
		usData = new float[us.sampleSize()];
	}

	public void run() {
		while (true) {
			us.fetchSample(usData, 0); 
			distance = (int) (usData[0] * 100.0); 			
			try {
				Thread.sleep(50);
			} catch (Exception e) {
				e.printStackTrace();
			} 
		}
	}
	public int getDistance() {
		return distance;
	}

}
