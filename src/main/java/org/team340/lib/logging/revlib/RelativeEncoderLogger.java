package org.team340.lib.logging.revlib;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

@CustomLoggerFor(RelativeEncoder.class)
public class RelativeEncoderLogger extends ClassSpecificLogger<RelativeEncoder> {

    private static final RelativeEncoderStruct struct = new RelativeEncoderStruct();

    public RelativeEncoderLogger() {
        super(RelativeEncoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, RelativeEncoder relativeEncoder) {
        backend.log("", relativeEncoder, struct);
    }

    private static class RelativeEncoderStruct implements Struct<RelativeEncoder> {

        @Override
        public Class<RelativeEncoder> getTypeClass() {
            return RelativeEncoder.class;
        }

        @Override
        public String getTypeName() {
            return "RelativeEncoder";
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
        public RelativeEncoder unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, RelativeEncoder value) {
            bb.putDouble(value.getPosition());
            bb.putDouble(value.getVelocity());
        }
    }
}
