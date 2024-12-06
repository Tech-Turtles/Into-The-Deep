package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {

    private Constants() {
        throw new AssertionError("Utility class");
    }

    // ------------------------------
    // Sensor Settings
    // ------------------------------
    public static double COLOR_SENSOR_STOP_DISTANCE = 0.4;

    // ------------------------------
    // Slide Mechanism Constants
    // ------------------------------
    public static double SLIDE_P = 0.028;
    public static double SLIDE_I = 0.028;
    public static double SLIDE_D = 0.00004;
    public static double SLIDE_FEEDFORWARD_EXPONENT = 0.5;
    public static double SLIDE_FEEDFORWARD_BASE = 0.4;
    public static double SLIDE_FEEDFORWARD_MAX = 0.58;

    public static double SLIDE_EXTENSION_LIMIT_TICKS = -1223;

    public static double HIGH_SPEC_EXT_SLIDE = -364;
    public static double HIGH_SAMPLE_EXT_SLIDE = SLIDE_EXTENSION_LIMIT_TICKS;
    public static double LOW_SAMPLE_EXT_SLIDE = -500;

    public static double SPECIMEN_LIMIT_TICKS = 364;
    public static double RETRACT_FUDGE_LIMIT_TICKS = 20;

    public static double SLIDE_TICKS_PER_ROTATION = 537.7;
    public static double SLIDE_HORIZONTAL_LIMIT_ROTATIONS = 1.55;

    // ------------------------------
    // Arm Mechanism Constants
    // ------------------------------
    public static int ARM_FEEDFORWARD_EXPONENT = 2;

    public static double ARM_HIGH_SPEC_PIVOT_ANGLE = 800;
    public static double ARM_HIGH_SAMPLE_PIVOT_ANGLE = 650;
    public static double ARM_LOW_SAMPLE_PIVOT_ANGLE = 500;
    public static double ARM_BUCKET_SAMPLE_ANGLE = -500;
    public static double ARM_VERTICAL_POSITION = 700;
    public static double ARM_HORIZONTAL_POSITION = -1500;
    public static double ARM_HIGH_CHAMBER_END_POSITION = -50;

    public static double ARM_FEEDFORWARD = 0.145;
    public static double ARM_FEEDFORWARD_MAX = 0.3;
    public static double ARM_ZERO_OFFSET = 1450;
    public static double ARM_TICKS_PER_DEGREE = 22.5096444;

    public static double ARM_HIGH_ANGLE_FEEDFORWARD_CUTOFF = 90;
    public static double ARM_LOW_ANGLE_FEEDFORWARD_CUTOFF = -10;
    public static double ARM_FEEDFORWARD_CUTOFF_COEFFICIENT = 0.2;

    public static double ARM_LOWER_LIMIT = -1667;
    public static double ARM_UPPER_LIMIT = 950;

    public static double ARM_WALL_SPECIMEN_POSITION = 200;

    public static double ARM_P = 0.008;
    public static double ARM_I = 0.02;
    public static double ARM_D = 0.00006;

    // ------------------------------
    // Drive Controls
    // ------------------------------
    public static double DRIVE_SLOW_MODE_MULTIPLIER = 0.4;

}
