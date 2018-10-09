package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Localizer {

	private static final int FORWARD_SPEED = 200;
	private static final int TURN_SPEED = 60;
	private static final int FORWARD_ACCELERATION = 2000;
	private static final int TURN_ACCELERATION = 2000;
	private static final double TILE_SIZE = 30.48;

	private static final EV3LargeRegulatedMotor leftMotor = Lab4.leftMotor;
	private static final EV3LargeRegulatedMotor rightMotor = Lab4.rightMotor;
	private static Odometer odo;
	private static double[] currentPosition;
	private static SampleProvider usAverage = Lab4.getUSAverage();
	private static float[] usData = Lab4.getUSData();
	private static SampleProvider color = Lab4.getColor();
	private static float[] colorBuffer = Lab4.getColorBuffer();

	private static int d = 33;
	private static int k = 2;
	private static int rrising = 20;
	private static int rfalling = 20;

	public static void localizeFE() throws OdometerExceptions {

		// Initialize variables
		double a1, a2, b1, b2, a, b, correction;
		a1 = a2 = b1 = b2 = a = b = 0;
		boolean a1set, a2set, b1set, b2set;
		a1set = a2set = b1set = b2set = false;
		int dist, lastdist;
		dist = lastdist = Integer.MAX_VALUE;

		// Get Odometer Instance
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			throw e;
		}

		// Start turning, this will be stopped when Falling Edges are detected.
		turnBy(1000, true);

		// Sample from Ultrasonic Sensor
		usAverage.fetchSample(usData, 0);
		dist = (int) (usData[0] * 100.00);

		// If we're already underneath the threshold, rotate until we are no longer
		// underneath it
		while (dist < d + k + rfalling || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (back wall) is found
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			if (dist <= d + k && lastdist <= d + k && !a1set) {
				Sound.beep();
				a1 = odo.getXYT()[2];
				a1set = true;
			}
			if (dist <= d - k && lastdist <= d - k && a1set && !a2set) {
				Sound.beep();
				a2 = odo.getXYT()[2];
				a2set = true;
			}
			if (a1set && a2set) {
				a = (a1 + a2) / 2;
				break;
			}
			lastdist = dist;
		}

		// Stop rotating and rotate in opposite direction
		leftMotor.stop(true);
		rightMotor.stop(false);
		turnBy(1000, false);

		while (dist < d + k + rfalling || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (right wall) is found.
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			if (dist <= d + k && lastdist <= d + k && !b1set) {
				Sound.beep();
				b1 = odo.getXYT()[2];
				b1set = true;
			}
			if (dist <= d - k && lastdist <= d + k && b1set && !b2set) {
				Sound.beep();
				b2 = odo.getXYT()[2];
				b2set = true;
			}
			if (b1set && b2set) {
				b = (b1 + b2) / 2;
				break;
			}
			lastdist = dist;
		}

		// Stop rotating.
		leftMotor.stop(true);
		rightMotor.stop(false);

		// Correct theta and orientate to 0.
		if (a < b) {
			correction = 45 - (a + b) / 2;

			// turnTo(180-(correction+odo.getXYT()[2]));
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			turnTo(0);

		}
		if (a >= b) {
			correction = 225 - (a + b) / 2;
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			turnTo(0);
			// odo.setTheta(odo.getXYT()[2] - (225 - (a+b)/2));
		}

	}

	public static void localizeRE() throws OdometerExceptions {

		// Initialize variables
		double a1, a2, b1, b2, a, b, correction;
		a1 = a2 = b1 = b2 = a = b = 0;
		boolean a1set, a2set, b1set, b2set;
		a1set = a2set = b1set = b2set = false;
		int dist, lastdist;
		dist = lastdist = 0;

		// Get Odometer Instance
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			throw e;
		}

		// Start turning, this will be stopped when Falling Edges are detected.
		turnBy(1000, true);

		// Sample from Ultrasonic Sensor
		usAverage.fetchSample(usData, 0);
		dist = (int) (usData[0] * 100.00);

		// If we're above the distance threshold, rotate until we are no longer
		// above it
		while (dist > d - k - rrising || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (back wall) is found
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			if (dist >= d - k && lastdist >= d - k && !b1set) {
				Sound.beep();
				b1 = odo.getXYT()[2];
				b1set = true;
			}
			if (dist >= d + k && lastdist >= d - k && b1set && !b2set) {
				Sound.beep();
				b2 = odo.getXYT()[2];
				b2set = true;
			}
			if (b1set && b2set) {
				b = (b1 + b2) / 2;
				break;
			}
			lastdist = dist;
		}

		// Stop rotating and rotate in opposite direction
		leftMotor.stop(true);
		rightMotor.stop(false);
		turnBy(1000, false);

		while (dist > d - k - rrising || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (right wall) is found.
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			if (dist >= d - k && lastdist >= d - k && !a1set) {
				Sound.beep();
				a1 = odo.getXYT()[2];
				a1set = true;
			}
			if (dist >= d + k && lastdist >= d + k && a1set && !a2set) {
				Sound.beep();
				a2 = odo.getXYT()[2];
				a2set = true;
			}
			if (a1set && a2set) {
				a = (a1 + a2) / 2;
				break;
			}
			lastdist = dist;
		}

		// Stop rotating.
		leftMotor.stop(true);
		rightMotor.stop(false);

		// Correct theta and orientate to 0.
		if (a < b) {
			correction = 45 - (a + b) / 2;

			// turnTo(180-(correction+odo.getXYT()[2]));
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			turnTo(0);

		}
		if (a >= b) {
			correction = 225 - (a + b) / 2;
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			turnTo(0);
		}

	}

	public static void travelTo(double x, double y) {
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		currentPosition = odo.getXYT();

		double deltaX = x * TILE_SIZE - currentPosition[0];
		double deltaY = y * TILE_SIZE - currentPosition[1];
		double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

		if (deltaX == 0 && deltaY != 0) {
			turnTo(deltaY < 0 ? 180 : 0);
		} else {
			double baseAngle = deltaX > 0 ? 90 : 270;
			double adjustAngle;
			if ((deltaY > 0 && deltaX > 0) || (deltaY < 0 && deltaX < 0)) {
				adjustAngle = -1 * Math.toDegrees(Math.atan(deltaY / deltaX));
			} else {
				adjustAngle = Math.toDegrees(Math.atan(Math.abs(deltaY) / Math.abs(deltaX)));
			}

			turnTo(baseAngle + adjustAngle);
		}
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		while (true) {
			currentPosition = odo.getXYT();

			deltaX = x * TILE_SIZE - currentPosition[0];
			deltaY = y * TILE_SIZE - currentPosition[1];
			distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
			
			if(distance < 1.0) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				turnTo(0);
			}
		}

	}

	public static void localizeColor() throws OdometerExceptions {
		boolean ySet, xSet;
		ySet = false; 
		xSet = false;

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			throw e;
		}

		double currentColor, lastColor;
		currentColor = 2.0;
		lastColor = 0.0;
		color.fetchSample(colorBuffer, 0);
		currentColor = colorBuffer[0];

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(FORWARD_ACCELERATION);
			motor.setSpeed(FORWARD_SPEED/2);
		}

		leftMotor.forward();
		rightMotor.forward();

		while (true) {

			color.fetchSample(colorBuffer, 0);
			currentColor = colorBuffer[0];
			
			if (currentColor - lastColor > 5 && !ySet) {
				Sound.beep();
				odo.setY(0);
				ySet = true;
				leftMotor.stop(true);
				rightMotor.stop(false);
				leftMotor.rotate(-1 * convertDistance(Lab4.getWheelRad(), 5), true);
				rightMotor.rotate(-1 * convertDistance(Lab4.getWheelRad(), 5), false);
				turnTo(90);
				leftMotor.forward();
				rightMotor.forward();
				lastColor = currentColor;
				continue;
			}

			
			
			if (currentColor - lastColor > 5 && ySet && !xSet) {
				Sound.beep();
				odo.setX(0);
				xSet = true;
				break;
			}
			
			lastColor = currentColor;

		}

		leftMotor.stop(true);
		rightMotor.stop(false);
		
		turnTo(0);
		
		leftMotor.rotate(convertDistance(Lab4.getWheelRad(), 5), true);
		rightMotor.rotate(convertDistance(Lab4.getWheelRad(), 5), false);
		
		

	}

	public static void turnBy(double theta, boolean clockwise) {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(TURN_ACCELERATION);
			motor.setSpeed(TURN_SPEED);
		}

		leftMotor.rotate((clockwise ? 1 : -1) * convertAngle(Lab4.getWheelRad(), Lab4.getTrack(), theta), true);
		rightMotor.rotate((clockwise ? -1 : 1) * convertAngle(Lab4.getWheelRad(), Lab4.getTrack(), theta), true);

	}

	/**
	 * This method makes the robot turn to the specified bearing.
	 * 
	 * @param theta Bearing for the robot to readjust its heading to.
	 */
	public static void turnTo(double theta) {

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(TURN_ACCELERATION);
			motor.setSpeed(TURN_SPEED);
		}

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
