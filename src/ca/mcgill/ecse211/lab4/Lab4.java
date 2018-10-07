// Lab2.java
package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Lab4 {

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	//Configuration Objects
	private static final double WHEEL_RAD = 2.09;
	private static final double TRACK = 12.70;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static int mapSelection;
	
	
	// Sensor Objects
	private static SampleProvider usDistance = new EV3UltrasonicSensor(usPort).getMode("Distance");
	private static float[] usData = new float[usDistance.sampleSize()];
	
	
	
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;
		mapSelection = 2;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete
																							// implementation
		Display odometryDisplay = new Display(lcd); // No need to change

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("       | Drive  ", 0, 2);
			lcd.drawString("       | in a   ", 0, 3);
			lcd.drawString("       | Map(#" + mapSelection +")", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
		if (buttonChoice == Button.ID_RIGHT) {
			// clear the display
			lcd.clear();

			// ask the user whether odometery correction should be run or not
			lcd.drawString("< Left |        ", 0, 0);
			lcd.drawString("  No   |        ", 0, 1);
			lcd.drawString(" corr- |        ", 0, 2);
			lcd.drawString(" ection|        ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			// spawn a new Thread to avoid SquareDriver.drive() from blocking
			(new Thread() {
				public void run() {
					//Localize
				}
			}).start();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
	
	public static double getWheelRad() {
		return WHEEL_RAD;
	}
	
	public static SampleProvider getUSDistance() {
		return usDistance;
	}
	
	public static float[] getUSData() {
		return usData;
	}
	
	public static double getTrack() {
		return TRACK;
	}
	
	public static int getMapSelection() {
		return mapSelection;
	}
}
