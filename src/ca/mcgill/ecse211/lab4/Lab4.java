// Lab3.java
package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab3.Display;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class runs the lab 4
 * 
 * @author Sophie Deng
 */

public class Lab4 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final Port usPort = LocalEV3.get().getPort("S4");
	static SensorModes usSensor = new EV3UltrasonicSensor(usPort);
	static SampleProvider usDistance = usSensor.getMode("Distance");
	static float[] usData = new float[usDistance.sampleSize()];

	private static final Port lightPort = LocalEV3.get().getPort("S1");
	static EV3ColorSensor myColor  = new EV3ColorSensor(lightPort); 
	static SampleProvider usIntensity = myColor.getRedMode();
	static float[]sampleColor = new float[usIntensity.sampleSize()]; 


	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 9.9;



	//final static UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);

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

			// ask the user whether the motors should do lab 4 or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("  Test | Drive  ", 0, 2);
			lcd.drawString("motors | and do ", 0, 3);
			lcd.drawString("       | lab 4  ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
//			// Float the motors
//			leftMotor.forward();
//			leftMotor.flt();
//			rightMotor.forward();
//			rightMotor.flt();
			
			Navigation navigation = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);
			
			navigation.turnTo(270);
			navigation.turnTo(135);
			navigation.turnTo(0);
			navigation.turnTo(45);

		} else {
			// clear the display
			lcd.clear();
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("rising | falling", 0, 2);
			lcd.drawString("edge   | edge   ", 0, 3);
			int buttonWait = Button.waitForAnyPress();
			
			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();


			if (buttonWait == Button.ID_LEFT) {
				// spawn a new Thread to avoid Navigation from blocking
				(new Thread() {

					/** Does the localization in rising edge
					 *
					 */
					public void run() {

						try {
							Navigation navigation = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);

							//USLocalization
							USLocalizer usLoc = new USLocalizer(odometer, usDistance, usData, navigation);
							usLoc.doLocalization(true);

							int buttonAngle = Button.waitForAnyPress();

							//move to 0,0
							boolean noLineH = true;

							while(noLineH) {
								navigation.moveForward();
								usIntensity.fetchSample(sampleColor, 0);
								float intensity = sampleColor[0];
								if(intensity < 0.4) {
									navigation.stop();
									noLineH = false;
								}
							}

							navigation.turnTo(90);

							boolean noLineV = true;
							while(noLineV) {
								navigation.moveForward();
								usIntensity.fetchSample(sampleColor, 0);
								float intensity = sampleColor[0];
								if(intensity < 0.4) {
									navigation.stop();
									noLineV = false;
								}
							}


							//do light localizer


						} catch (OdometerExceptions e) {
							e.printStackTrace();
						}

					}
				}).start();
			}
			else {
				// spawn a new Thread to avoid Navigation from blocking
				(new Thread() {

					/** Does the localization in falling edge
					 *
					 */
					public void run() {
						try {
							Navigation navigation = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);

							//USLocalization
							USLocalizer usLoc = new USLocalizer(odometer, usDistance, usData, navigation);
							usLoc.doLocalization(false);

							int buttonAngle = Button.waitForAnyPress();

							//move to 0,0
							boolean noLineH = true;

							while(noLineH) {
								navigation.moveForward();
								usIntensity.fetchSample(sampleColor, 0);
								float intensity = sampleColor[0];
								if(intensity < 0.4) {
									navigation.stop();
									noLineH = false;
								}
							}

							navigation.turnTo(90);

							boolean noLineV = true;
							while(noLineV) {
								navigation.moveForward();
								usIntensity.fetchSample(sampleColor, 0);
								float intensity = sampleColor[0];
								if(intensity < 0.4) {
									navigation.stop();
									noLineV = false;
								}
							}

							//do light localizer


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
