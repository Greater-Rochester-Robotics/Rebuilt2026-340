package org.team340.lib.logging.revlib;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

@CustomLoggerFor(SparkAbsoluteEncoder.class)
public class SparkAbsoluteEncoderLogger extends ClassSpecificLogger<SparkAbsoluteEncoder> {

    private static final SparkAbsoluteEncoderStruct struct = new SparkAbsoluteEncoderStruct();

    public SparkAbsoluteEncoderLogger() {
        super(SparkAbsoluteEncoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkAbsoluteEncoder absoluteEncoder) {
        backend.log("", absoluteEncoder, struct);
    }

    private static class SparkAbsoluteEncoderStruct implements Struct<SparkAbsoluteEncoder> {

        @Override
        public Class<SparkAbsoluteEncoder> getTypeClass() {
            return SparkAbsoluteEncoder.class;
        }

        @Override
        public String getTypeName() {
            return "SparkAbsoluteEncoder";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 2;
        }

        @Override
        public String getSchema() {
            return "double position; double velocity;";
        }

        @Override
        public SparkAbsoluteEncoder unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, SparkAbsoluteEncoder value) {
            bb.putDouble(value.getPosition());
            bb.putDouble(value.getVelocity());
        }
    }
}
