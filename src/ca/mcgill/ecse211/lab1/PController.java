package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements the Wall Follower for Lab1 on the EV3 platform in PController.
 * 
 * @author Sophie and Edris
 */

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 150;
	private static final int FILTER_OUT = 3;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	//constructor
	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	/** Function to decide how the wheels turn
	 * @param distance to wall
	 */
	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 200 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			System.out.println("Filter Control is " + filterControl);
			filterControl++;
		} else if (distance >= 200) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = 200; //max distance so the error doesn't go crazy 
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		//make sure the distance has been recorded
		if (this.distance == distance) {
			int error = bandCenter - distance;
			System.out.println("Distance to wall is " + distance);
			if (Math.abs(error) <= this.bandWidth) {
				//if within distance, go straight
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.leftMotor.forward();
				WallFollowingLab.rightMotor.forward();
			}
			else if (error > 0) {
				//too close, right motor roll back to turn in place
				System.out.println("Too Close");
				int correction = error*5;
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + correction);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + correction);
				WallFollowingLab.leftMotor.forward();
				WallFollowingLab.rightMotor.backward();
			}
			else {
				//too far, slow and fast depending on the error
				System.out.println("Too Far");
				int correction = Math.abs(error)*2;
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + correction);
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - correction);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}

		}
	}


	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
