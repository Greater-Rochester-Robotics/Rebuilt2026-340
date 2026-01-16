package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(Pigeon2.class)
public class Pigeon2Logger extends ClassSpecificLogger<Pigeon2> {

    private static final Pigeon2Struct struct = new Pigeon2Struct();

    public Pigeon2Logger() {
        super(Pigeon2.class);
    }

    @Override
    public void update(EpilogueBackend backend, Pigeon2 pigeon2) {
        backend.log("", pigeon2, struct);
    }

    private static class Pigeon2Struct implements Struct<Pigeon2> {

        @Override
        public Class<Pigeon2> getTypeClass() {
            return Pigeon2.class;
        }

        @Override
        public String getTypeName() {
            return "Pigeon2";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 7;
        }

        @Override
        public String getSchema() {
            return (
                "double yaw; "
                + "double pitch; "
                + "double roll; "
                + "double angularVelX; "
                + "double angularVelY; "
                + "double angularVelZ; "
                + "double temperature;"
            );
        }

        @Override
        public Pigeon2 unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, Pigeon2 value) {
            registry.computeIfAbsent(value, mappingFunction).accept(bb);
        }

        private static final Map<Pigeon2, Consumer<ByteBuffer>> registry = new HashMap<>();
        private static final Function<Pigeon2, Consumer<ByteBuffer>> mappingFunction = value -> {
            var yaw = value.getYaw(false);
            var pitch = value.getPitch(false);
            var roll = value.getRoll(false);
            var angularVelX = value.getAngularVelocityXWorld(false);
            var angularVelY = value.getAngularVelocityYWorld(false);
            var angularVelZ = value.getAngularVelocityZWorld(false);
            var temperature = value.getTemperature(false);

            BaseStatusSignal[] signals = { yaw, pitch, roll, angularVelX, angularVelY, angularVelZ, temperature };

            return bb -> {
                BaseStatusSignal.refreshAll(signals);
                bb.putDouble(yaw.getValueAsDouble());
                bb.putDouble(pitch.getValueAsDouble());
                bb.putDouble(roll.getValueAsDouble());
                bb.putDouble(angularVelX.getValueAsDouble());
                bb.putDouble(angularVelY.getValueAsDouble());
                bb.putDouble(angularVelZ.getValueAsDouble());
                bb.putDouble(temperature.getValueAsDouble());
            };
        };
    }
}
