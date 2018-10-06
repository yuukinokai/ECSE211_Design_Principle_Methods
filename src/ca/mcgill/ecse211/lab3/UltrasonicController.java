package ca.mcgill.ecse211.lab3;

/**
 * This class is the ultrasonic controller, it reads the distance
 */

public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}
