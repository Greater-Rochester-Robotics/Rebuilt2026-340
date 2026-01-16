package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import java.nio.ByteBuffer;

@CustomLoggerFor(ADIS16470_IMU.class)
public class ADIS16470Logger extends ClassSpecificLogger<ADIS16470_IMU> {

    private static final ADIS16470Struct struct = new ADIS16470Struct();

    public ADIS16470Logger() {
        super(ADIS16470_IMU.class);
    }

    @Override
    public void update(EpilogueBackend backend, ADIS16470_IMU adis16470) {
        backend.log("", adis16470, struct);
    }

    private static class ADIS16470Struct implements Struct<ADIS16470_IMU> {

        @Override
        public Class<ADIS16470_IMU> getTypeClass() {
            return ADIS16470_IMU.class;
        }

        @Override
        public String getTypeName() {
            return "ADIS16470";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 6;
        }

        @Override
        public String getSchema() {
            return (
                "double yaw; "
                + "double pitch; "
                + "double roll; "
                + "double yawRate; "
                + "double pitchRate; "
                + "double rollRate;"
            );
        }

        @Override
        public ADIS16470_IMU unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, ADIS16470_IMU value) {
            var yawAxis = value.getYawAxis();
            var pitchAxis = value.getPitchAxis();
            var rollAxis = value.getRollAxis();

            bb.putDouble(value.getAngle(yawAxis));
            bb.putDouble(value.getAngle(pitchAxis));
            bb.putDouble(value.getAngle(rollAxis));
            bb.putDouble(value.getRate(yawAxis));
            bb.putDouble(value.getRate(pitchAxis));
            bb.putDouble(value.getRate(rollAxis));
        }
    }
}
