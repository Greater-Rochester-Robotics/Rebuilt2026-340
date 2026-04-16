package org.team340.robot;

import com.ctre.phoenix6.CANBus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double VOLTAGE = 12.0;

    // Controller ports
    public static final int DRIVER = 0;

    /**
     * The RobotMap class defines CAN IDs, CAN bus names, DIO/PWM/PH/PCM channel
     * IDs, and other relevant identifiers for addressing robot hardware.
     */
    public static final class RobotMap {

        public static final CANBus CANBus = new CANBus("canivore");

        // Swerve

        public static final int FL_MOVE = 2;
        public static final int FL_TURN = 3;
        public static final int FR_MOVE = 4;
        public static final int FR_TURN = 5;
        public static final int BL_MOVE = 6;
        public static final int BL_TURN = 7;
        public static final int BR_MOVE = 8;
        public static final int BR_TURN = 9;

        public static final int FL_ENCODER = 10;
        public static final int FR_ENCODER = 11;
        public static final int BL_ENCODER = 12;
        public static final int BR_ENCODER = 13;

        public static final int CANANDGYRO = 14;

        // Shooter

        public static final int SHOOTER_PORT_UPPER_MOTOR = 20;
        public static final int SHOOTER_PORT_LOWER_MOTOR = 21;
        public static final int SHOOTER_STARBOARD_UPPER_MOTOR = 22;
        public static final int SHOOTER_STARBOARD_LOWER_MOTOR = 23;

        // Hood

        public static final int HOOD_MOTOR = 30;

        // Indexer

        public static final int INDEXER_PORT_UPPER_LEAD_MOTOR = 40;
        public static final int INDEXER_PORT_LOWER_FOLLOW_MOTOR = 41;
        public static final int INDEXER_STARBOARD_UPPER_FOLLOW_MOTOR = 42;
        public static final int INDEXER_STARBOARD_LOWER_FOLLOW_MOTOR = 43;

        // Intake

        public static final int INTAKE_PIVOT_MOTOR = 50;
        public static final int INTAKE_ROLLER_LEAD_MOTOR = 51;
        public static final int INTAKE_ROLLER_FOLLOW_MOTOR = 52;
        public static final int INTAKE_WCP_THROUGHBORE_POWERED_BY_CANCODER_FOR_HALF_INCH_HEX = 53;

        // Lights

        public static final int LIGHTS = 1;
    }
}
