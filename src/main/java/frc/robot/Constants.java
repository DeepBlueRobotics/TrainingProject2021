// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Motor {
        public static final int kSparkMax = 1;
    }

    public static final class Controller 
    {
        public static final int kPort = 2;

        // Buttons and triggers
        public static final int ARM = 1; // button for arm
        public static final int CALIBRATE = 2; // Button to calibrate position
        public static final int B = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int LT = 7;
        public static final int RT = 8;
        public static final int BACK = 9;
        public static final int START = 10;
    }
}
