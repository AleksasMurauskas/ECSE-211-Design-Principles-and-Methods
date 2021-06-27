package Navigation;
/**
 * ObstacleAvoidance class
 * Skeleton class taken from tutorial code
 * Expanded on extensively
 *  
 */
import Display.Log;
import Navigation.Navigator;

public class ObstacleAvoidance extends Thread{

	Navigator nav;
	boolean safe;
	
	public ObstacleAvoidance(Navigator nav){
		this.nav = nav;
		safe = false;
	}
	/**
	 * run method activates when a 
	 *  
	 * */
	public void run(){
		Log.log(Log.Sender.avoidance,"avoiding obstacle!");
		nav.setSpeeds(0, 0);
		/**
		 * Obtains the odometer values
		 *  
		 * */
		double currX =nav.odometer.getX();
		double currY =nav.odometer.getY();
		double currT =nav.odometer.getAng();
		/**
		 * The below if statements determine the cardinal direction the robot is facing, and then determiens if it should avoid the object to the right or left
		 */
		if(currT < 135 && currT > 45) { //Facing North
			if(currX>450) {
				avoidLeft();
			}
			else {
				avoidRight();
			}
		}
		else if((currT>0 && currT < 45)||(currT<360 && currT < (360-45))) {//Faces west
			if(currY>450) {
				avoidRight();
			}else{
				avoidLeft();
			}
		}
		else if(currT>135 && currT < 225) { //Faces East
			if(currY>450) {
				avoidLeft();
			}else{
				avoidRight();
			}
		}
		else { //Faces South
			if(currX>450) {
				avoidRight();
			}else{
				avoidLeft();
			}
		}
		/**
		 * Updates the log
		 * */
		Log.log(Log.Sender.avoidance,"obstacle avoided!");
		safe = true;
	}
	/**
	 * avoidLeft orders the robot to take a left turn around the object
	 * 
	 */
	public void avoidLeft() {
		Main.rightMotor.setSpeed(100);
		Main.leftMotor.setSpeed(100);

		Main.rightMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 90.0), true);
	    Main.leftMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 90.0), false);//First turn
	   
	    Main.leftMotor.rotate(convertDistance(Main.WHEEL_RADIUS, 40), true); // Drives forward 
	    Main.rightMotor.rotate(convertDistance(Main.WHEEL_RADIUS, 40), false);
	    
	    Main.leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 90.0), true);//Second turn
	    Main.rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 90.0), false);

	    Main.leftMotor.rotate(convertDistance(Main.WHEEL_RADIUS, 30), true); //Drives past the wall
	    Main.rightMotor.rotate(convertDistance(Main.WHEEL_RADIUS, 30), false);
	}
	/**
	 * avoidLeft orders the robot to take a right turn around the object
	 * 
	 */
	public void avoidRight() {
		
		Main.leftMotor.setSpeed(100);
		Main.rightMotor.setSpeed(100);

	    Main.leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 90.0), true);
	    Main.rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 90.0), false);//First turn

	    Main.leftMotor.rotate(convertDistance(Main.WHEEL_RADIUS, 40), true); //Drive forward
	    Main.rightMotor.rotate(convertDistance(Main.WHEEL_RADIUS, 40), false);

	    Main.leftMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 90.0), true); //Second turn
	    Main.rightMotor.rotate(+convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 90.0), false);

	    Main.leftMotor.rotate(convertDistance(Main.WHEEL_RADIUS, 30), true); //Drive forward past wall
	    Main.rightMotor.rotate(convertDistance(Main.WHEEL_RADIUS, 30), false);
	}
	public boolean resolved() {
		return safe;
	}
	
	 private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }
	 private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	 }
}
