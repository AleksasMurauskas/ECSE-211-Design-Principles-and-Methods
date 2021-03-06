package ca.mcgill.ecse211.lab5;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class ColorClassification {

	/**
	 * setup the motor for the sensor
	 */
	private static final EV3LargeRegulatedMotor sideMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	
	/**
	 * Light sensor object
	 */
	private EV3ColorSensor colorSensor;
	private SampleProvider rgbValue;

	/**
	 * Float arrays for Color data
	 */
	private float[] colorData;

	/**
	 *  Float arrays for red, green, blue, and yellow RGB Mean Values
	 */
	private static final float[] RED={0.96918374f,0.21552089f,0.11901724f};
	private static final float[] GREEN={0.23248355f,0.9499019f,0.20874791f};
	//private static final float[] BLUE={0.18840f,0.64278f,0.74252f};
	//private static final float[] BLUE = {0.00122f,0.430341f,0.902665f};
	//private static final float[] BLUE = {0.23767194f,0.84434474f,0.48017082f};
	private static final float[] BLUE = {0.2791057f,0.74138427f,0.6102413f};
	private static final float[] YELLOW={0.85795154f,0.4986058f,0.12356515f};
	
	/**
	 * 2D Float Array store the information in one list for iteration
	 */
	private static final float[][] COLOR_LIST= {RED,GREEN,BLUE,YELLOW};
	
	/**
	 *  tolerance interval for classifying a color
	 */
	private static final float ERROR=0.1f;
	
	/**
	 * the speed of the rotation of the sensor
	 */
	private static final int ROTATION_SPEED=30;
	
	
	/**
	 * the angle of the rotation of the sensor
	 */
	private static final int ROTATION_DEGREE=90;
	
	/**
	 * the max time of detection we will perform if we get a unmatched color
	 */
	private static final int MAX_DETECTION_TIMES= 30;
	
	/**
	 * Constructor
	 * @param colorSensor
	 */
	public ColorClassification(EV3ColorSensor colorSensor) {
		this.colorSensor = colorSensor;
		rgbValue = colorSensor.getRGBMode();
		colorData = new float[rgbValue.sampleSize()];
	}


	/**
	 * This method allows to detect the color of the can
	 * @return (integer representing the color)
	 */
	public int findColor() {
		//System.out.println("Color");
		// a integer representing the color we detect
		//-1 means no match color find
		// 0 means red, 1 means green, 2 means blue and 3 means yellow
		int color;
		
		//a counter that keep track how many detection & rotation we have make 
		int counter=0;
		sideMotor.setSpeed(ROTATION_SPEED);
		sideMotor.rotateTo(ROTATION_DEGREE,true);
		//match the color we detect with the sample colors stored in our software
		do {	
			//sideMotor.stop();
			color = matchColor( fetch() );			
			counter++;      		//the counter increased by one once we called the match color method
			//turnRight();			//turn right and get ready for another match color 
		} while (color == -1 && counter<MAX_DETECTION_TIMES);
		
		//turn the sensor back to its initial position
		backToInitial(counter);
		//System.out.println(color);
		//return the color we detect
		return color;		
	}

	
	/**
	 * This method allows to collect RGB values
	 * @return (array containing RGB values) 
	 */

	public float[] fetch() {
		rgbValue.fetchSample(colorData, 0);
		return colorData;
	}

	
	/**
	 * This method allows to match the readings and determine the color detected
	 * @return (integer representing color) 0-red 1-green 2-blue 3-yellow
	 * 
	 */
	public static int matchColor(float array[]) {
		
		//store the normalized RGB data in the array
		float normalizedColor[]=normalize(array);
		/*if((normalizedColor[0] > 2*normalizedColor[1])&&(normalizedColor[0] > 2*normalizedColor[2])) {
			return 0;
		}else if((normalizedColor[1] > 2*normalizedColor[0])&&(normalizedColor[1] > 2*normalizedColor[2])) {
			return 1;
		}else if((!(normalizedColor[2] > 2*normalizedColor[1]))&&(normalizedColor[2] > 2*normalizedColor[0])) {
			return 2;
		}else if((!(normalizedColor[0] > 2*normalizedColor[1]))&&(normalizedColor[0] > 2*normalizedColor[2])) {
			return 3;
		}*/
		

		//loop through the color list to see if they matches
		for (int i=0; i<4; i++) {
			//System.out.println(normalizedColor[0]+" "+normalizedColor[1]+" "+ normalizedColor[2]);
			//see if the value of the normalized RGB data and our pre-stored data is within a tolerable interval
			if (withinInterval(normalizedColor,COLOR_LIST[i],ERROR)){
				return i;
			}
		}
		//if does not match any of the colors, return -1
		return -1;
	}
	
	
	
	/**
	 * This method allow us to get the normalized RGB data from initial data
	 * @param array of RGB initial data 
	 * @return an array where the normalized RGB data stored
	 */
	public static float[] normalize(float array[]) {		
		float sqrAverage=(float)Math.sqrt((Math.pow(array[0], 2)+Math.pow(array[1], 2)+Math.pow(array[2], 2)));
		float R=array[0]/sqrAverage;
		float G=array[1]/sqrAverage;
		float B=array[2]/sqrAverage;	
		float nomalizedColor[]= {R,G,B};
		return nomalizedColor;
	}
	
	
/**
 * This method allow us to find whether one array is within interval of another
 * @param array1 
 * @param array2 
 * @param tolerace which means the max difference the value in the two arrays can have 
 * @return boolean whether the value in the two arrays are close enough
 */
	public static boolean withinInterval(float array1[], float array2[],float tolerace) {		
		if (Math.abs(array1[0]-array2[0])<=tolerace && Math.abs(array1[1]-array2[1])<=tolerace && Math.abs(array1[2]-array2[2])<=tolerace) {
			//System.out.println(array1[0]+" "+array1[1]+" "+array1[2] );
			return true;
		}
		return false;
	}
	
	
	/**
	 * This method make the sensor turn right for a fixed degree
	 */
	public static void turnRight(){
		sideMotor.setSpeed(ROTATION_SPEED);
		sideMotor.rotate(-ROTATION_DEGREE);
		
	}
	
	/**
	 * This method brings the sensor back to its initial position
	 * @param counter represent how many right turn we performed
	 */
	public static void backToInitial(int counter){
		sideMotor.setSpeed(ROTATION_SPEED*2);
		sideMotor.rotateTo(0,false);
	}
	
}






