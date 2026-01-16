package org.team340.lib.logging.reduxlib;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroStatus;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

@CustomLoggerFor(Canandgyro.class)
public class CanandgyroLogger extends ClassSpecificLogger<Canandgyro> {

    private static final CanandgyroStruct struct = new CanandgyroStruct();

    public CanandgyroLogger() {
        super(Canandgyro.class);
    }

    @Override
    public void update(EpilogueBackend backend, Canandgyro canandgyro) {
        backend.log("", canandgyro, struct);
    }

    private static class CanandgyroStruct implements Struct<Canandgyro> {

        @Override
        public Class<Canandgyro> getTypeClass() {
            return Canandgyro.class;
        }

        @Override
        public String getTypeName() {
            return "Canandgyro";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 6 + CanandgyroStatus.struct.getSize() + kSizeBool;
        }

        @Override
        public String getSchema() {
            return (
                "double yaw; "
                + "double pitch; "
                + "double roll; "
                + "double yawVelocity; "
                + "double pitchVelocity; "
                + "double rollVelocity; "
                + "CanandgyroStatus status;"
            );
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] { CanandgyroStatus.struct };
        }

        @Override
        public Canandgyro unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, Canandgyro value) {
            bb.putDouble(value.getYaw());
            bb.putDouble(value.getPitch());
            bb.putDouble(value.getRoll());
            bb.putDouble(value.getAngularVelocityYaw());
            bb.putDouble(value.getAngularVelocityPitch());
            bb.putDouble(value.getAngularVelocityRoll());
            CanandgyroStatus.struct.pack(bb, value.getStatus());
        }
    }
}
