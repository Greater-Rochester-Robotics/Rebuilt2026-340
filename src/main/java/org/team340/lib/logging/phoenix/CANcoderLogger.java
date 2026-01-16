package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(CANcoder.class)
public class CANcoderLogger extends ClassSpecificLogger<CANcoder> {

    private static final CANcoderStruct struct = new CANcoderStruct();

    public CANcoderLogger() {
        super(CANcoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANcoder cancoder) {
        backend.log("", cancoder, struct);
    }

    private static class CANcoderStruct implements Struct<CANcoder> {

        @Override
        public Class<CANcoder> getTypeClass() {
            return CANcoder.class;
        }

        @Override
        public String getTypeName() {
            return "CANcoder";
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
        public CANcoder unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, CANcoder value) {
            registry.computeIfAbsent(value, mappingFunction).accept(bb);
        }

        private static final Map<CANcoder, Consumer<ByteBuffer>> registry = new HashMap<>();
        private static final Function<CANcoder, Consumer<ByteBuffer>> mappingFunction = value -> {
            var position = value.getPosition(false);
            var velocity = value.getVelocity(false);

            BaseStatusSignal[] signals = { position, velocity };

            return bb -> {
                BaseStatusSignal.refreshAll(signals);
                bb.putDouble(position.getValueAsDouble());
                bb.putDouble(velocity.getValueAsDouble());
            };
        };
    }
}
