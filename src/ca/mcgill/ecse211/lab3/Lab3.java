// Lab3.java
package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab3.Display;
import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class runs the lab 3
 * 
 * @author Sophie
 */

public class Lab3 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 9.9;

	static Port usPort = LocalEV3.get().getPort("S4");
	static SensorModes usSensor = new EV3UltrasonicSensor(usPort);
	static SampleProvider usDistance = usSensor.getMode("Distance");
	static float[] usData = new float[usDistance.sampleSize()];
	final static UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);

	/** Main entry point - instantiate objects used and set up sensor
	 * @param args not used
	 */

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		//OdometryCorrection odometryCorrection = new OdometryCorrection();
		Display odometryDisplay = new Display(lcd); // No need to change


		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" Float | Drive  ", 0, 2);
			lcd.drawString("motors | and do ", 0, 3);
			lcd.drawString("       | lab 3  ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			// Float the motors
			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();

			// Display changes in position as wheels are (manually) moved

		} else {
			// clear the display
			lcd.clear();
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("    no | with  ", 0, 2);
			lcd.drawString("correc | correc ", 0, 3);
			int buttonWait = Button.waitForAnyPress();
			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();



			if (buttonChoice == Button.ID_LEFT) {
				// spawn a new Thread to avoid Navigation from blocking
				(new Thread() {

					/** Runs the navigation - decides what the waypoint is
					 *
					 */
					public void run() {
						//waypoints
						int wayPoint1[][] = new int[][] {{ 0 , 2 }, { 1 , 1 }, { 2 , 2 }, { 2, 1 }, { 1 , 0 }};

						try {

							Navigation navigation = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);

							for (int i = 0; i < 5; i ++) {
								navigation.travelTo(wayPoint1[i][0], wayPoint1[i][1]);
							}


						} catch (OdometerExceptions e) {
							e.printStackTrace();
						}

					}
				}).start();
			}
			else {
				// spawn a new Thread to avoid Navigation from blocking
				(new Thread() {

					/** Runs the navigation - decides what the waypoint is
					 *
					 */
					public void run() {
						//waypoints
						int wayPoint1[][] = new int[][] {{ 0 , 2 }, { 1 , 1 }, { 2 , 2 }, { 2, 1 }, { 1 , 0 }};

						try {

							Navigation navigation = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);

							for (int i = 0; i < 5; i ++) {
								navigation.travelToWithCorrection(wayPoint1[i][0], wayPoint1[i][1]);
							}


						} catch (OdometerExceptions e) {
							e.printStackTrace();
						}

					}
				}).start();
			}


		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
