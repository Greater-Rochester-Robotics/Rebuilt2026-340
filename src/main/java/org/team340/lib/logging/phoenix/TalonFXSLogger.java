package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

@CustomLoggerFor(TalonFXS.class)
public class TalonFXSLogger extends ClassSpecificLogger<TalonFXS> {

    private static final TalonFXSStruct struct = new TalonFXSStruct();

    public TalonFXSLogger() {
        super(TalonFXS.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFXS talonFXS) {
        backend.log("", talonFXS, struct);
    }

    private static class TalonFXSStruct implements Struct<TalonFXS> {

        @Override
        public Class<TalonFXS> getTypeClass() {
            return TalonFXS.class;
        }

        @Override
        public String getTypeName() {
            return "TalonFXS";
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
        public TalonFXS unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, TalonFXS value) {
            registry.computeIfAbsent(value, mappingFunction).accept(bb);
        }

        private static final Map<TalonFXS, Consumer<ByteBuffer>> registry = new HashMap<>();
        private static final Function<TalonFXS, Consumer<ByteBuffer>> mappingFunction = value -> {
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
