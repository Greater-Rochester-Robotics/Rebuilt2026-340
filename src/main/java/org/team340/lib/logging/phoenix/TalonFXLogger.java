package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

    private static final TalonFXStruct struct = new TalonFXStruct();

    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX talonFX) {
        backend.log("", talonFX, struct);
    }

    private static class TalonFXStruct implements Struct<TalonFX> {

        @Override
        public Class<TalonFX> getTypeClass() {
            return TalonFX.class;
        }

        @Override
        public String getTypeName() {
            return "TalonFX";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 7;
        }

        @Override
        public String getSchema() {
            return (
                "double closedLoopReference; "
                + "double deviceTemp; "
                + "double motorVoltage; "
                + "double position; "
                + "double statorCurrent; "
                + "double supplyCurrent; "
                + "double velocity;"
            );
        }

        @Override
        public TalonFX unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, TalonFX value) {
            registry.computeIfAbsent(value, mappingFunction).accept(bb);
        }

        private static final Map<TalonFX, Consumer<ByteBuffer>> registry = new HashMap<>();
        private static final Function<TalonFX, Consumer<ByteBuffer>> mappingFunction = value -> {
            var closedLoopReference = value.getClosedLoopReference(false);
            var deviceTemp = value.getDeviceTemp(false);
            var motorVoltage = value.getMotorVoltage(false);
            var position = value.getPosition(false);
            var statorCurrent = value.getStatorCurrent(false);
            var supplyCurrent = value.getSupplyCurrent(false);
            var velocity = value.getVelocity(false);

            BaseStatusSignal[] signals = {
                closedLoopReference,
                deviceTemp,
                motorVoltage,
                position,
                statorCurrent,
                supplyCurrent,
                velocity
            };

            return bb -> {
                BaseStatusSignal.refreshAll(signals);
                bb.putDouble(closedLoopReference.getValueAsDouble());
                bb.putDouble(deviceTemp.getValueAsDouble());
                bb.putDouble(motorVoltage.getValueAsDouble());
                bb.putDouble(position.getValueAsDouble());
                bb.putDouble(statorCurrent.getValueAsDouble());
                bb.putDouble(supplyCurrent.getValueAsDouble());
                bb.putDouble(velocity.getValueAsDouble());
            };
        };
    }
}
