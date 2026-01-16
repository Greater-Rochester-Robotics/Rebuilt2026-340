package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(CANdi.class)
public class CANdiLogger extends ClassSpecificLogger<CANdi> {

    private static final CANdiStruct struct = new CANdiStruct();

    public CANdiLogger() {
        super(CANdi.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANdi candi) {
        backend.log("", candi, struct);
    }

    private static class CANdiStruct implements Struct<CANdi> {

        @Override
        public Class<CANdi> getTypeClass() {
            return CANdi.class;
        }

        @Override
        public String getTypeName() {
            return "CANdi";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 4 + kSizeBool * 2;
        }

        @Override
        public String getSchema() {
            return (
                "double pwm1Position; "
                + "double pwm1Velocity; "
                + "double pwm2Position; "
                + "double pwm2Velocity; "
                + "bool s1Closed; "
                + "bool s2Closed;"
            );
        }

        @Override
        public CANdi unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, CANdi value) {
            registry.computeIfAbsent(value, mappingFunction).accept(bb);
        }

        private static final Map<CANdi, Consumer<ByteBuffer>> registry = new HashMap<>();
        private static final Function<CANdi, Consumer<ByteBuffer>> mappingFunction = value -> {
            var pwm1Position = value.getPWM1Position(false);
            var pwm1Velocity = value.getPWM1Velocity(false);
            var pwm2Position = value.getPWM2Position(false);
            var pwm2Velocity = value.getPWM2Velocity(false);
            var s1Closed = value.getS1Closed(false);
            var s2Closed = value.getS2Closed(false);

            BaseStatusSignal[] signals = { pwm1Position, pwm1Velocity, pwm2Position, pwm2Velocity, s1Closed, s2Closed };

            return bb -> {
                BaseStatusSignal.refreshAll(signals);
                bb.putDouble(pwm1Position.getValueAsDouble());
                bb.putDouble(pwm1Velocity.getValueAsDouble());
                bb.putDouble(pwm2Position.getValueAsDouble());
                bb.putDouble(pwm2Velocity.getValueAsDouble());
                bb.put((byte) (s1Closed.getValue() ? 1 : 0));
                bb.put((byte) (s2Closed.getValue() ? 1 : 0));
            };
        };
    }
}
