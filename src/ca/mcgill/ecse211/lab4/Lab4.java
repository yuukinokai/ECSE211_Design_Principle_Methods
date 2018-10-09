// Lab3.java
package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.*;
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
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 

		final Display odometryDisplay = new Display(lcd);


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
			// Float the motors
			//			leftMotor.forward();
			//			leftMotor.flt();
			//			rightMotor.forward();
			//			rightMotor.flt();
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			Navigation navigation = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);
			
			//test the angles
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
				lcd.clear();
				Navigation navigation = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);

				//USLocalization
				USLocalizer usLoc = new USLocalizer(usDistance, usData, navigation);
				usLoc.doLocalization(true);

				int buttonAngle = Button.waitForAnyPress();

				//move to 0,0
				boolean noLineH = true;
				lcd.clear();
				navigation.moveForward();
				while(noLineH) {

					usIntensity.fetchSample(sampleColor, 0);
					float intensity = sampleColor[0]*100;
					//System.out.println(intensity);
					if(intensity < 33) {
						navigation.stop();
						noLineH = false;
					}
				}

				navigation.turnTo(90);

				boolean noLineV = true;
				navigation.moveForward();
				while(noLineV) {
					usIntensity.fetchSample(sampleColor, 0);
					float intensity = sampleColor[0]*100;
					if(intensity < 33) {
						navigation.stop();
						noLineV = false;
					}
				}

				navigation.turnTo(0);
				//do light localizer

				LightLocalizer lightLoc = new LightLocalizer(usIntensity, sampleColor, navigation);
				lightLoc.doLocalization();

			}
			else {

				Navigation navigation = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);

				//USLocalization
				USLocalizer usLoc = new USLocalizer(usDistance, usData, navigation);
				usLoc.doLocalization(false);

				int buttonAngle = Button.waitForAnyPress();

				//move to 0,0
				boolean noLineH = true;
				lcd.clear();
				navigation.moveForward();
				while(noLineH) {

					usIntensity.fetchSample(sampleColor, 0);
					float intensity = sampleColor[0]*100;
					System.out.println(intensity);
					if(intensity < 33) {
						navigation.stop();
						noLineH = false;
					}
				}

				navigation.turnTo(90);

				boolean noLineV = true;
				navigation.moveForward();
				while(noLineV) {
					usIntensity.fetchSample(sampleColor, 0);
					float intensity = sampleColor[0]*100;
					if(intensity < 33) {
						navigation.stop();
						noLineV = false;
					}
				}

				//do light localizer
				LightLocalizer lightLoc = new LightLocalizer(usIntensity, sampleColor, navigation);
				lightLoc.doLocalization();
			}


		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
