package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

/**
 * This class implements the Wall Follower for Lab1 on the EV3 platform in PController.
 * 
 * @author Sophie Deng
 */

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	/** Function to decide how the wheels turn
	 * @param distance to wall
	 */
	@Override
	public void processUSData(int distance) {
		System.out.println("Distance to wall is " + distance);
		this.distance = distance;
		int error = bandCenter - distance;
		if (Math.abs(error) <= bandwidth) {
			//if within error, go straight
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
		}
		else if (error > 0) {
			//too close, slow down right motor and accelerate left motor
			System.out.println("Too Close");
			WallFollowingLab.leftMotor.setSpeed(motorHigh+100); //inner turns are sharper
			WallFollowingLab.rightMotor.setSpeed(motorLow-30);
		}
		else {
			//too far, slow down right motor and accelerate left motor
			System.out.println("Too Far");
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
		}
		//move forward
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
