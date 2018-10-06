package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
//import lejos.hardware.motor.Motor;
import lejos.robotics.RegulatedMotor;
//import ca.mcgill.ecse211.odometer.Odometer;
//import ca.mcgill.ecse211.odometer.OdometerExceptions;
//import lejos.hardware.Button;
//import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class does the navigation to the desired points
 * 
 * @author Sophie
 */
public class Navigation {

	private static Odometer odometer;
	private static double position[];
	// private double newheading;
	private final static int FORWARD_SPEED = 150;
	private final static int ROTATE_SPEED = 90;
	private final static double TILE_SIZE = 30.48;
	private final static double TRESHOLD = 20.0;


	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	public static double wheelRadius = 2.2;
	static double TRACK = 9.9;

	public static double dx = 0;
	public static double dy = 0;
	
	private boolean navigating = false;

	/**
	 * Constructor
	 */
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double wheelRad, double track) throws OdometerExceptions {
		Navigation.odometer = Odometer.getOdometer();
		Navigation.leftMotor = leftMotor;
		Navigation.rightMotor = rightMotor;
		wheelRadius = wheelRad;
		TRACK = track;
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(500);
		}
	}
	
	/**
	 * @return if the robot is navigating
	 */
	public boolean isNavigating(){
			
		return navigating;
			
	}

	/**
	 * This method makes the robot travel to a certain point,  
	 * it detects walls and goes around the block
	 * 
	 * @param xf
	 * @param yf
	 * 
	 * @throws OdometerExceptions
	 */
	public void travelToWithCorrection(double xf, double yf) throws OdometerExceptions {

		xf *= TILE_SIZE;
		yf *= TILE_SIZE;
		double initial_distance = turnTo(xf, yf);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(wheelRadius, initial_distance), true);
		rightMotor.rotate(convertDistance(wheelRadius, initial_distance), true);


		while (isNavigating()) {
			boolean checkWall = Lab3.usPoller.detectWall(TRESHOLD);				
			if(checkWall) {
				goAround();
				initial_distance = turnTo(xf, yf);

				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);

				leftMotor.rotate(convertDistance(wheelRadius, initial_distance), true);
				rightMotor.rotate(convertDistance(wheelRadius, initial_distance), false);
				
			}
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			}
		}

		return;

	}

	/**
	 * This method makes the robot travel to a certain point,
	 * it does not detect walls
	 * 
	 * @param xf
	 * @param yf
	 * 
	 * @throws OdometerExceptions
	 */
	public void travelTo(double xf, double yf) throws OdometerExceptions {

		xf *= TILE_SIZE;
		yf *= TILE_SIZE;

		double initial_distance = turnTo(xf, yf);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(wheelRadius, initial_distance), true);
		rightMotor.rotate(convertDistance(wheelRadius, initial_distance), false);

		return;

	}
	/**
	 * This method turns the robot towards the point and returns the distance
	 * 
	 * @param xf
	 * @param yf
	 * 
	 * @return distance
	 */
	private static double turnTo(double xf, double yf) {
		double xi, yi, turning_angle = 0;

		double initialDistance = 0;

		//get current position
		position = odometer.getXYT();
		xi = position[0];
		yi = position[1];

		//determine final position

		dx = xf - xi;
		dy = yf - yi;

		initialDistance = compute_distance(dx, dy);
		turning_angle = getAngle(dx, dy);

		double angleDif = turning_angle - position[2];

		//compute minimum angle
		if(angleDif > 180) {
			angleDif *= 1;
			angleDif = (360 - turning_angle) + position[2];
		}
		else if(angleDif < 180) {
			angleDif *= -1;
			angleDif = (360 - position[2]) + turning_angle;
		}

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		if(angleDif > 0) {
			leftMotor.rotate(convertAngle(wheelRadius, TRACK, Math.abs(angleDif)), true);
			rightMotor.rotate(-convertAngle(wheelRadius, TRACK, Math.abs(angleDif)), false);
		}
		else if(angleDif < 0) {
			leftMotor.rotate(-convertAngle(wheelRadius, TRACK, Math.abs(angleDif)), true);
			rightMotor.rotate(convertAngle(wheelRadius, TRACK, Math.abs(angleDif)), false);
		}

		return initialDistance;
	}

	/**
	 * This method computes the distance between two points
	 * 
	 * @param dx
	 * @param dy
	 * @return the distance
	 */
	private static double compute_distance(double dx, double dy) {
		return Math.sqrt(dx * dx + dy * dy);
	}


	/**
	 * This method gives the heading of the next way point, that is, what angle
	 * should the robot be at in order to arrive at the location quickly
	 * 
	 * @param dx
	 * @param dy
	 * @return angle in degrees
	 */
	private static double getAngle(double dx, double dy) {
		double angle = 0;

		if (dy > 0) { //if the robot should go up
			angle = Math.atan( Math.abs(dx) / Math.abs(dy));
			if (dx < 0) { //if it should go left
				angle = Math.PI * 2 - angle;
			}
			else if (dx == 0) {
				angle = 0;
			}
		} 
		else if (dy < 0) { //if the robot should go down
			angle = (Math.atan(Math.abs(dx) / Math.abs(dy)) + Math.PI * 0.5);
			if (dx < 0) {
				angle += Math.PI * 0.5;
			}
			else if (dx == 0) {
				angle = Math.PI;
			}
		}
		else { //if dy is 0, the angle should be pi/2 or 3pi/2
			if(dx < 0) {
				angle = Math.PI * 1.5;
			}
			else{
				angle = Math.PI * 0.5;
			}
		}
		return angle * 180 / Math.PI;
	}

	/**
	 * This method computes how much the wheels should turn to get to an angle
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}


	/**
	 * This method makes the robot go around a block
	 * 
	 */
	private void goAround() {
		//turn right
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(wheelRadius, TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(wheelRadius, TRACK, 90.0), false);

		//go straight
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(wheelRadius, 1 * TILE_SIZE), true);
		rightMotor.rotate(convertDistance(wheelRadius, 1 * TILE_SIZE), false);

		//turn left
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(-convertAngle(wheelRadius, TRACK, 90.0), true);
		rightMotor.rotate(convertAngle(wheelRadius, TRACK, 90.0), false);

		//go straight
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(wheelRadius, 1 * TILE_SIZE), true);
		rightMotor.rotate(convertDistance(wheelRadius, 1 * TILE_SIZE), false);
	}
}
