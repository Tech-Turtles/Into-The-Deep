package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {

    // ------------------------------
    // Sensor Settings
    // ------------------------------
    public static double COLOR_SENSOR_STOP_DISTANCE = 0.4;
    // ------------------------------
    // Slide Mechanism Constants
    // ------------------------------
    public static double SLIDE_P = 0.025;
    public static double SLIDE_I = 0.01;
    public static double SLIDE_D = 0.00001;
    public static double SLIDE_FEEDFORWARD_EXPONENT = 0.5;
    public static double SLIDE_FEEDFORWARD_BASE = 0.4;
    public static double SLIDE_FEEDFORWARD_MAX = 0.58;
    public static double SLIDE_EXTENSION_LIMIT_TICKS = -1223;
    public static double HIGH_SPEC_EXT_SLIDE = -400;
    public static double HIGH_SPEC_WALL_EXT_SLIDE = -425;
    //public static double HIGH_SPEC_PLACE_EXT = -200;
    public static double HIGH_SAMPLE_EXT_SLIDE = SLIDE_EXTENSION_LIMIT_TICKS;
    public static double LOW_SAMPLE_EXT_SLIDE = -525;
    public static double SPECIMEN_LIMIT_TICKS = 364;
    public static double SLIDE_WALL_SPEC_INTAKE_EXT = 0;
    public static double RETRACT_FUDGE_LIMIT_TICKS = 20;
    public static double SLIDE_TICKS_PER_ROTATION = 537.7;
    public static double SLIDE_HORIZONTAL_LIMIT_ROTATIONS = 1.55;
    public static double SLIDE_SPECIMEN_RETRACT_TICKS = -134;
    // ------------------------------
    // Arm Mechanism Constants
    // ------------------------------
    public static int ARM_FEEDFORWARD_EXPONENT = 2;
    public static double ARM_HIGH_SPEC_PIVOT_ANGLE = 800;
    public static double ARM_HIGH_SAMPLE_PIVOT_ANGLE = 400;
    public static double ARM_LOW_SAMPLE_PIVOT_ANGLE = 350;
    public static double ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE = 380;
    public static double ARM_BUCKET_SAMPLE_ANGLE = -800;
    public static double ARM_VERTICAL_POSITION = 700;
    public static double ARM_HORIZONTAL_POSITION = -1500;
    public static double ARM_HIGH_CHAMBER_END_POSITION = 50;
    public static double ARM_WALL_SPEC_INTAKE_ANGLE = -600;

    public static double TELE_ARM_WALL_SPEC_INTAKE_ANGLE = -600;
    public static double ARM_FEEDFORWARD = 0.145;
    public static double ARM_FEEDFORWARD_MAX = 0.3;
    public static double ARM_ZERO_OFFSET = 1450;
    public static double ARM_TICKS_PER_DEGREE = 22.5096444;
    public static double ARM_HIGH_ANGLE_FEEDFORWARD_CUTOFF = 90;
    public static double ARM_LOW_ANGLE_FEEDFORWARD_CUTOFF = -10;
    public static double ARM_FEEDFORWARD_CUTOFF_COEFFICIENT = 0.2;
    public static double ARM_LOWER_LIMIT = -1667;
    public static double ARM_UPPER_LIMIT = 950;
    public static double ARM_P = 0.008;
    public static double ARM_I = 0.02;
    public static double ARM_D = 0.00006;
    public static double AUTO_ARM_P_FUDGE_FACTOR = -0.00625;
    public static double AUTO_ARM_I_FUDGE_FACTOR = 0.00;
    public static double AUTO_ARM_D_FUDGE_FACTOR = 0.0002;
    public static double AUTO_SLIDE_P_FUDGE_FACTOR = 0.00;
    public static double AUTO_SLIDE_I_FUDGE_FACTOR = 0.002;
    public static double AUTO_SLIDE_D_FUDGE_FACTOR = 0.00024;
    public static double AUTO_maxWheelVel = 30;
    public static double AUTO_minProfileAccel = -30;
    public static double AUTO_maxProfileAccel = 30;
    public static double AUTO_maxAngVel_Mult = 1;
    public static double maxAngAccel_Mult = 1;
    public static double AUTO_axialGain = 1.5;
    public static double AUTO_lateralGain = 2.0;
    public static double AUTO_headingGain = 3.0;


    // ------------------------------
    // Drive Controls
    // ------------------------------
    public static double DRIVE_SLOW_MODE_MULTIPLIER = 0.4;
    // ------------------------------
    // Robot Measurements
    // ------------------------------
    public static double robotW = 17.8;
    public static double robotH = 16;
    public static double robotHalfW = robotW / 2.0;

    //-------------------------------------------------------------------------
    public static double blue1Angle = 20;
    public static double BLUE_1_X_REL_VALUE = 19;
    public static double BLUE_1_Y_REL_VALUE = -8.5  + 3;
    public static double BLUE_1_DEGREE_REL_VALUE = 0;
    public static double blue1ToHPZoneAngle = -120;
    public static double BLUE_1_X_TO_HPZONE_REL_VALUE = -14;
    public static double BLUE_1_Y_TO_HPZONE_REL_VALUE = -50;
    public static double BLUE_1_DEGREE_TO_HPZONE_REL_VALUE = 140;
    public static double blue2Angle = 30;
    public static double BLUE_2_X_REL_VALUE = 19;
    public static double BLUE_2_Y_REL_VALUE = 19+50;
    public static double BLUE_2_DEGREE_REL_VALUE = -(BLUE_1_DEGREE_TO_HPZONE_REL_VALUE) + 35 -2-1;
    public static double BLUE_2_DEGREE_TO_HPZONE_REL_VALUE = -(BLUE_1_DEGREE_TO_HPZONE_REL_VALUE) + 25;
    public static double blue2ToHPZoneAngle = 30;
    public static double blue3Angle = 45;
    public static double BLUE_3_X_REL_VALUE = 19;
    public static double BLUE_3_Y_REL_VALUE = 19+40;
    public static double BLUE_3_DEGREE_REL_VALUE = -(BLUE_2_DEGREE_TO_HPZONE_REL_VALUE) +10-20-5-15+10-3.5+1.5;
    public static double BLUE_3_TOTAL_EXTENSION = -((SLIDE_HORIZONTAL_LIMIT_ROTATIONS * SLIDE_TICKS_PER_ROTATION) + 155) ;
    public static double BLUE_3_PARTIAL_EXTENSION = 430+50;
    public static double blue3ToHPZoneAngle = 45;
    public static double BLUE_1_TOTAL_EXTENSION = -670;
    public static double BLUE_1_PARTIAL_EXTENSION = 430;
    public static double BLUE_2_TOTAL_EXTENSION = -750;
    public static double BLUE_2_PARTIAL_EXTENSION = 430;
    public static double blue3Ext = -500;
    public static double SLIDE_CEILING_MAX = 1;
    public static double BLUE_1_SLIDE_CEILING = 0.4;
    public static double AUTO_DRIVING_ARM_HOROZONTAL_POSITION = -1350;
    //-------------------------------------------------------------------------

    private Constants() {
        throw new AssertionError("Utility class");
    }
}
