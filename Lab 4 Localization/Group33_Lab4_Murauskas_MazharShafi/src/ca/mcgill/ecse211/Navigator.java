package ca.mcgill.ecse211;
/**
 * Navigation Class
 * Subclass of Basic Navigator
 * Obtained from Tutorial Code
 * Expanded upon for lab 4
 */


public class Navigator extends BasicNavigator {
	/**
	 * Key state machine enumerations instantiated 
	 * 
	 * */
	enum State {
		INIT, TURNING, TRAVELLING,
	};
	State state;
	boolean notDone =true;
	private boolean isNavigating = false;
	private double destx, desty;
	final static int SLEEP_TIME = 50;
	UltrasonicPoller usSensor;

	public Navigator(Odometer odo, UltrasonicPoller usSensor) {
		super(odo);
		this.usSensor = usSensor;
	}
	/**
	 * travelTo calls the superclass's method unless Avoid is true
	 * 
	 * */
	public void travelTo(double x, double y, boolean avoid) {
		if (avoid) {
			destx = x;
			desty = y;
			isNavigating = true;
		} else {
			super.travelTo(x, y);
		}
	}
	private void updateTravel() {
		double minAng;
		minAng = getDestAngle(destx, desty);
		super.turnTo(minAng,false);
		this.setSpeeds(FAST, FAST);
	}
	
	/**
	 * Run
	 * Key method, contains the infinite loop that operates the robot and state machine
	 */
	public void run() {
		state = State.INIT;
		while (notDone) {
			switch (state) {
			case INIT:
				/**
				 * Initial State and ending state
				 */
				if (isNavigating) {
					state = State.TURNING;
				}
				break;
			case TURNING:
				/**
				 * State set to when the robot must correct it's angle 
				 * 
				 * */
				double destAngle = getDestAngle(destx, desty);
				turnTo(destAngle);
				if(facingDest(destAngle)){
					setSpeeds(0,0);
					state = State.TRAVELLING;
				}
				break;
			case TRAVELLING:
				/**
				 * State Set when the robot travels to it's destination 
				 * 
				 * */
				} if (!checkIfDone(destx, desty)) {
					updateTravel();
				} else { 
					/**
					 * Robot has completed it's journey
					 * 
					 * */
					setSpeeds(0, 0);
					isNavigating = false;
					state = State.INIT;
				}
				break;
			}
			/**
			 * Updates the Log
			 * 
			 * */
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	
	/**
	 * checkEmergency polls the US Sensor for an obstacle
	 * 
	 * */
	private boolean checkEmergency() {
		return usSensor.getDistance() < 15;
	}
	/**
	 * turnTo turns the robot to the desired angle, using the minimal angle
	 * @param desired angle 
	 */
	private void turnTo(double angle) {
		double error;
		error = angle - this.odometer.getAng();
		/**
		 * Minimal Angle found
		 *  
		 * */
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
	/**
	 * goForward orders the motors to go straight forward a specified distance 
	 * @param distance to be traveled
	 * @param avoid state to be passed to travelTo
	 * */
	public void goForward(double distance, boolean avoid) {
		double x = odometer.getX()
				+ Math.cos(Math.toRadians(this.odometer.getAng())) * distance;
		double y = odometer.getY()
				+ Math.sin(Math.toRadians(this.odometer.getAng())) * distance;
		this.travelTo(x, y, avoid);
	}
	/**
	 * isTravelling returns the state of the boolean isNavigating 
	 *  
	 */
	public boolean isTravelling() {
		return isNavigating;
	}
}
