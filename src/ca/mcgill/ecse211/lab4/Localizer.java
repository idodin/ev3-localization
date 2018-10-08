package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Localizer {
	
	private static final int FORWARD_SPEED = 200;
	private static final EV3LargeRegulatedMotor leftMotor = Lab4.leftMotor;
	private static final EV3LargeRegulatedMotor rightMotor = Lab4.rightMotor;
	private static Odometer odo;
	private static double[] currentPosition;
	private static SampleProvider usDistance = Lab4.getUSDistance();
	private static float[] usData = Lab4.getUSData();
	private static int d = 0;
	private static int k = 0;
	
	public static void localizeFE() throws OdometerExceptions {
		
		// Initialize variables 
		int a1, a2, b1, b2, a, b;
		a1 = a2 = b1 = b2 = a = b = 0;
		boolean a1set, a2set, b1set, b2set;
		a1set = a2set = b1set = b2set = false;
		
		// Get Odometer Instance
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			throw e;
		}
		
		
		// Start turning, this will be stopped when Falling Edges are detected.
		turnBy(1000, true);
		
		// Sample from Ultrasonic Sensor
		usDistance.fetchSample(usData, 0);
		int dist = (int) (usData[0] * 100.00);
		
		// If we're already underneath the threshold, rotate until we are no longer underneath it
		while(dist < d+k) {
			usDistance.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}
		
		// Keep rotating until falling edge (back wall) is found
		while(true) {
			usDistance.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			if(dist <= d+k && !a1set) {
				a1 = dist;
				a1set = true;
			}
			if(dist <= d-k && a1set && !a2set) {
				a2 = dist;
				a2set = true;
			}
			if(a1set && a2set) {
				a = (a1 + a2)/2;
				break;
			}
		}
		
		// Stop rotating and rotate in opposite direction
		leftMotor.stop(true);
		rightMotor.stop(false);
		turnBy(1000, false);
		
		// Keep rotating until falling edge (right wall) is found.
		while(true) {
			usDistance.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			if(dist <= d+k && !b1set) {
				b1 = dist;
				b1set = true;
			}
			if(dist <= d-k && b1set && !b2set) {
				b2 = dist;
				b2set = true;
			}
			if(b1set && b2set) {
				b = (b1 + b2)/2;
				break;
			}
		}
		
		// Stop rotating.
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		// Correct theta and orientate to 0.
		if(a<b) {
			odo.setTheta(odo.getXYT()[2] - (45 - (a+b)/2));
			turnTo(0);
		}
		if(a>=b) {
			odo.setTheta(odo.getXYT()[2] - (225 - (a+b)/2));
			turnTo(0);
		}
		
	}
	
	public static void turnBy(double theta, boolean clockwise) {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		//set acceleration?
		
		leftMotor.rotate((clockwise? 1 : -1) * convertAngle(Lab4.getWheelRad(), Lab4.getTrack(), theta), true);
		rightMotor.rotate((clockwise? -1 : 1) * convertAngle(Lab4.getWheelRad(), Lab4.getTrack(), theta), false);
	
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
	}

	/**
	 * This method makes the robot turn to the specified bearing.
	 * 
	 * @param theta
	 *            Bearing for the robot to readjust its heading to.
	 */
	public static void turnTo(double theta) {

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);

		currentPosition = odo.getXYT();

		double deltaT = (((theta - currentPosition[2]) % 360) + 360) % 360;

		if (deltaT < 180) {
			leftMotor.rotate(convertAngle(Lab4.getWheelRad(), Lab4.getTrack(), deltaT), true);
			rightMotor.rotate(-convertAngle(Lab4.getWheelRad(), Lab4.getTrack(), deltaT), false);
		} else {
			leftMotor.rotate(-convertAngle(Lab4.getWheelRad(), Lab4.getTrack(), 360 - deltaT), true);
			rightMotor.rotate(convertAngle(Lab4.getWheelRad(), Lab4.getTrack(), 360 - deltaT), false);
		}

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of an angle to the rotation of each wheel
	 * needed to rotate that angle
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
