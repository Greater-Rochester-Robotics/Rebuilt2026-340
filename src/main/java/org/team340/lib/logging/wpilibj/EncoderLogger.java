package org.team340.lib.logging.wpilibj;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.Encoder;
import java.nio.ByteBuffer;

@CustomLoggerFor(Encoder.class)
public class EncoderLogger extends ClassSpecificLogger<Encoder> {

    private static final EncoderStruct struct = new EncoderStruct();

    public EncoderLogger() {
        super(Encoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, Encoder encoder) {
        backend.log("", encoder, struct);
    }

    private static class EncoderStruct implements Struct<Encoder> {

        @Override
        public Class<Encoder> getTypeClass() {
            return Encoder.class;
        }

        @Override
        public String getTypeName() {
            return "Encoder";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 2 + kSizeInt32;
        }

        @Override
        public String getSchema() {
            return "double distance; double rate; int32 raw;";
        }

        @Override
        public Encoder unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, Encoder value) {
            bb.putDouble(value.getDistance());
            bb.putDouble(value.getRate());
            bb.putInt(value.getRaw());
        }
    }
}
