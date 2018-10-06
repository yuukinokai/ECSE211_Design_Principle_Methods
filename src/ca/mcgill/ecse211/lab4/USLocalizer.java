package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class does the ultrasonic localization
 * 
 * @author Sophie Deng
 */
public class USLocalizer {

	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private Navigation navigation;

	private static final int ROTATE_SPEED = 90;
	private static final double WALLDISTANCE = 30;
	private static final double NOISE = 3;

	/**
	 * Constructor
	 */
	public USLocalizer(Odometer odo, SampleProvider usSensor, float[] usData, Navigation nav){
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.navigation = nav;
	}

	/**
	 * Perform localization, robot should be facing north
	 * @param true for rising edge, false for falling edge
	 */
	public void doLocalization(boolean risingEdge) {
		double angleA, angleB, angleZero;
		//get the two angles
		if(risingEdge) {
			angleA = getAngleARisingEdge();
			angleB = getAngleBRisingEdge();
		}
		else {
			angleA = getAngleAFallingEdge();
			angleB = getAngleBFallingEdge();

		}
		//get offset angle
		angleZero = getAngleZero(angleA, angleB);

		//get real angle
		double currentAngle = odo.getXYT()[2];
		double realAngle = (currentAngle + angleZero) % 360;
		if(realAngle < 0) {
			realAngle += 360;
		}

		//set real angle and turn to 0
		odo.setTheta(realAngle);
		navigation.turnTo(0);
	}

	/**
	 * get angleA for rising edge
	 */
	private double getAngleARisingEdge() {
		while(getDistance() > WALLDISTANCE + NOISE) {
			//rotate robot counterclockwise
			navigation.rotateWheels(-ROTATE_SPEED, ROTATE_SPEED);
		}
		while(getDistance() < WALLDISTANCE) {
			//rotate robot
			navigation.rotateWheels(-ROTATE_SPEED, ROTATE_SPEED);
		}

		//stop
		navigation.stop();
		return odo.getXYT()[2];
	}
	/**
	 * get angleB for rising edge
	 */
	private double getAngleBRisingEdge() {
		while(getDistance() > WALLDISTANCE + NOISE) {
			//rotate robot clockwise
			navigation.rotateWheels(ROTATE_SPEED, -ROTATE_SPEED);
		}
		while(getDistance() < WALLDISTANCE) {
			//rotate robot
			navigation.rotateWheels(ROTATE_SPEED, -ROTATE_SPEED);
		}

		//stop
		navigation.stop();

		return odo.getXYT()[2];
	}
	/**
	 * get angleA for falling edge
	 */
	private double getAngleAFallingEdge() {
		while(getDistance() < WALLDISTANCE + NOISE) {
			//rotate robot clockwise
			navigation.rotateWheels(ROTATE_SPEED, -ROTATE_SPEED);
		}
		while(getDistance() > WALLDISTANCE) {
			//rotate robot
			navigation.rotateWheels(ROTATE_SPEED, -ROTATE_SPEED);
		}

		//stop
		navigation.stop();
		return odo.getXYT()[2];
	}
	/**
	 * get angleB for falling edge
	 */
	private double getAngleBFallingEdge() {
		while(getDistance() < WALLDISTANCE + NOISE) {
			//rotate robot counterclockwise
			navigation.rotateWheels(-ROTATE_SPEED, ROTATE_SPEED);
		}
		while(getDistance() > WALLDISTANCE) {
			//rotate robot
			navigation.rotateWheels(-ROTATE_SPEED, ROTATE_SPEED);
		}

		//stop
		navigation.stop();
		return odo.getXYT()[2];
	}

	/**
	 * get distance from the ultrasonic sensor
	 */
	private float getDistance() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;
		return distance;
	}

	/**
	 * get offset angle
	 */
	private double getAngleZero(double angleA, double angleB) {
		double angleZero = 0;
		if(angleA > angleB) {
			angleZero = 255 - (angleA + angleB)/2.0;
		}
		else {
			angleZero = 45 - (angleA + angleB)/2.0;
		}
		return angleZero;
	}

}
