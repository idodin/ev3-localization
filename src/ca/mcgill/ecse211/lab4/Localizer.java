package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class contains methods for Rising Edge and Falling Edge Ultrasonic Localization as well as 
 * Color Sensor Localization.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 *
 */
public class Localizer {

	private static final int FORWARD_SPEED = 100;
	private static final int TURN_SPEED = 60;
	private static final int FORWARD_ACCELERATION = 2000;
	private static final int TURN_ACCELERATION = 2000;

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
	private static int rfalling = 30;

	/**
	 * Run Falling Edge Localization with the Ultrasonic Sesnor.
	 * @throws OdometerExceptions
	 */
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
			if (dist > 3 && dist <= d + k && lastdist <= d + k && !a1set) {
				Sound.beep();
				a1 = odo.getXYT()[2];
				a1set = true;
			}
			if (dist > 3 && dist <= d - k && lastdist <= d - k && a1set && !a2set) {
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

		stopMotors();
		turnBy(1000, false);

		while (dist < d + k + rfalling || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (right wall) is found.
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			if (dist > 3 && dist <= d + k && lastdist <= d + k && !b1set) {
				Sound.beep();
				b1 = odo.getXYT()[2];
				b1set = true;
			}
			if (dist > 3 && dist <= d - k && lastdist <= d + k && b1set && !b2set) {
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

		stopMotors();

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

	/**
	 * Run Rising Edge Localization with the EV3UltrasonicSensor.
	 * @throws OdometerExceptions
	 */
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

		stopMotors();
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

		stopMotors();

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

	/**
	 * Localize with line detection using the EV3ColorSensor
	 * @throws OdometerExceptions
	 */
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

		setSpeedAccel(FORWARD_SPEED, FORWARD_ACCELERATION);

		leftMotor.forward();
		rightMotor.forward();

		while (true) {

			color.fetchSample(colorBuffer, 0);
			currentColor = colorBuffer[0];

			if (currentColor - lastColor > 5 && !ySet) {
				Sound.beep();
				odo.setY(0);
				ySet = true;
				stopMotors();
				leftMotor.rotate(-1 * convertDistance(Lab4.getWheelRad(), 5), true);
				rightMotor.rotate(-1 * convertDistance(Lab4.getWheelRad(), 5), false);
				turnTo(90);
				setSpeedAccel(FORWARD_SPEED, FORWARD_ACCELERATION);
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

		stopMotors();

		turnTo(0);
		setSpeedAccel(FORWARD_SPEED, FORWARD_ACCELERATION);

		leftMotor.rotate(convertDistance(Lab4.getWheelRad(), 5), true);
		rightMotor.rotate(convertDistance(Lab4.getWheelRad(), 5), false);

	}

	/**
	 * Stop both motors at the same time.
	 */
	private static void stopMotors() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * Set Speed and Acceleration for both motors.
	 * @param speed
	 * @param accel
	 */
	private static void setSpeedAccel(int speed, int accel) {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(accel);
			motor.setSpeed(speed);
		}
	}

	/**
	 * Make the Robot turn by the specified amount in the specified direction.
	 * @param theta
	 * @param clockwise - true if clockwise, false if anticlockwise
	 */
	public static void turnBy(double theta, boolean clockwise) {
		setSpeedAccel(TURN_SPEED, TURN_ACCELERATION);

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
