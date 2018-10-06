package ca.mcgill.ecse211.lab4;

import lejos.hardware.*;
import lejos.robotics.SampleProvider;

/**
 * This class does the light localization
 * 
 * @author Sophie Deng
 */
public class LightLocalizer {
	private Odometer odometer;
	private Navigation navigation;
	private double[] angleData;
	
	private SampleProvider colourSensor;
	private float[] colourData;
	
	private static int ROTATE_SPEED = 90;
	private static double SENSOR_DISTANCE = 7;
	

	
	/**
	 * Constructor
	 */
	public LightLocalizer(Odometer odo, SampleProvider cSensor, float[] cData, Navigation nav) {
		this.odometer = odo;
		this.colourSensor = cSensor;
		this.colourData = cData;
		this.navigation = nav;
		this.angleData = new double[4];
	}
	
	/**
	 * Perform localization, robot should be facing north at (0,0)
	 */
	public void doLocalization() {
		collectAngleData();
		correctPosition();
		double currentX = odometer.getXYT()[0];
		double currentY = odometer.getXYT()[1];
		
		double dx = 0 - currentX;
		double dy = 0 - currentY;
		navigation.travelTo(dx, dy);
		navigation.turnTo(0);
	}
	
	/**
	 * correct x and y position
	 */
	private void correctPosition() {
		double deltaY = (angleData[3] - angleData[1]);
		double deltaX = (angleData[2] - angleData[0]);
		
		double xPosition = SENSOR_DISTANCE * Math.cos(Math.PI*deltaX/(2*180));
		double yPosition = SENSOR_DISTANCE * Math.cos(Math.PI*deltaY/(2*180));
		
		odometer.setX(-1 * xPosition);
		odometer.setY(-1 * yPosition);
	}

	/**
	 * collect angles at which the lines are
	 */
	private void collectAngleData() {
		navigation.rotateWheels(-ROTATE_SPEED, ROTATE_SPEED);
		int count = 0;
		while(count != 4) {
			colourSensor.fetchSample(colourData, 0);
			if(colourData[0] < 0.4) {
				Sound.buzz();
				angleData[count] = odometer.getXYT()[2];
				count++;
			}
		}
		navigation.stop();		
	}
}
