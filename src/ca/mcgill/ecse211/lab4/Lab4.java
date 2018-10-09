// Lab2.java
package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

public class Lab4 {

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	// Configuration Objects
	private static final double WHEEL_RAD = 2.09;
	private static final double TRACK = 12.70;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port colorPort = LocalEV3.get().getPort("S4");
	private static int mapSelection;

	// Sensor Objects
	private static SampleProvider usDistance = new EV3UltrasonicSensor(usPort).getMode("Distance");
	private static SampleProvider usAverage = new MeanFilter(usDistance, 5);
	private static float[] usData = new float[usAverage.sampleSize()];
	
	private static SampleProvider color = new EV3ColorSensor(colorPort).getMode("ColorID");
	private static float[] colorBuffer = new float[color.sampleSize()];
	
	
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;
		mapSelection = 2;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // No need to change

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left   | Right >  ", 0, 0);
			lcd.drawString("         |          ", 0, 1);
			lcd.drawString("  Rising | Falling  ", 0, 2);
			lcd.drawString("    Edge | Edge     ", 0, 3);
			lcd.drawString("         |          ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		// Start odometer and display threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		if (buttonChoice == Button.ID_RIGHT) {
			// spawn a new Thread to avoid SquareDriver.drive() from blocking
			(new Thread() {
				public void run() {
					try {
						Localizer.localizeFE();
					} catch (OdometerExceptions e) {
						e.printStackTrace();
					}
					int buttonChoice;
					do {
						// clear the display
						lcd.clear();

						// ask the user whether the motors should drive in a square or float
						lcd.drawString("<        | Right >     ", 0, 0);
						lcd.drawString("         |             ", 0, 1);
						lcd.drawString("         | Color       ", 0, 2);
						lcd.drawString("         | Localization", 0, 3);
						lcd.drawString("         |             ", 0, 4);

						buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
					} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
					if(buttonChoice == Button.ID_RIGHT) {
						lcd.clear();
						try{
							Localizer.localizeColor();
						} catch (OdometerExceptions e) {
							e.printStackTrace();
						}
					}
				}
			}).start();
		}
		if (buttonChoice == Button.ID_LEFT) {
			// spawn a new Thread to avoid SquareDriver.drive() from blocking
			(new Thread() {
				public void run() {
					try {
						Localizer.localizeRE();
					} catch (OdometerExceptions e) {
						e.printStackTrace();
					}
					int buttonChoice;
					do {
						// clear the display
						lcd.clear();

						// ask the user whether the motors should drive in a square or float
						lcd.drawString("<        | Right >     ", 0, 0);
						lcd.drawString("         |             ", 0, 1);
						lcd.drawString("         | Color       ", 0, 2);
						lcd.drawString("         | Localization", 0, 3);
						lcd.drawString("         |             ", 0, 4);

						buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
					} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
					if(buttonChoice == Button.ID_RIGHT) {
						lcd.clear();
						try{
							Localizer.localizeColor();
						} catch (OdometerExceptions e) {
							e.printStackTrace();
						}
					}
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
	
	public static SampleProvider getUSAverage() {
		return usAverage;
	}

	public static SampleProvider getColor() {
		return color;
	}

	public static float[] getColorBuffer() {
		return colorBuffer;
	}
}
