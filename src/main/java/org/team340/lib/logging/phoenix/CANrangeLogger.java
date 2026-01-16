package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(CANrange.class)
public class CANrangeLogger extends ClassSpecificLogger<CANrange> {

    private static final CANrangeStruct struct = new CANrangeStruct();

    public CANrangeLogger() {
        super(CANrange.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANrange canrange) {
        backend.log("", canrange, struct);
    }

    private static class CANrangeStruct implements Struct<CANrange> {

        @Override
        public Class<CANrange> getTypeClass() {
            return CANrange.class;
        }

        @Override
        public String getTypeName() {
            return "CANrange";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 3 + kSizeBool;
        }

        @Override
        public String getSchema() {
            return "double ambientSignal; double distance; bool isDetected; double signalStrength;";
        }

        @Override
        public CANrange unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, CANrange value) {
            registry.computeIfAbsent(value, mappingFunction).accept(bb);
        }

        private static final Map<CANrange, Consumer<ByteBuffer>> registry = new HashMap<>();
        private static final Function<CANrange, Consumer<ByteBuffer>> mappingFunction = value -> {
            var ambientSignal = value.getAmbientSignal(false);
            var distance = value.getDistance(false);
            var isDetected = value.getIsDetected(false);
            var signalStrength = value.getSignalStrength(false);

            BaseStatusSignal[] signals = { ambientSignal, distance, isDetected, signalStrength };

            return bb -> {
                BaseStatusSignal.refreshAll(signals);
                bb.putDouble(ambientSignal.getValueAsDouble());
                bb.putDouble(distance.getValueAsDouble());
                bb.put((byte) (isDetected.getValue() ? 1 : 0));
                bb.putDouble(signalStrength.getValueAsDouble());
            };
        };
    }
}
