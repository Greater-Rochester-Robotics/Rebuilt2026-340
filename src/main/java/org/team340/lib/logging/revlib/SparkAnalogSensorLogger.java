package org.team340.lib.logging.revlib;

import com.revrobotics.spark.SparkAnalogSensor;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

@CustomLoggerFor(SparkAnalogSensor.class)
public class SparkAnalogSensorLogger extends ClassSpecificLogger<SparkAnalogSensor> {

    private static final SparkAnalogSensorStruct struct = new SparkAnalogSensorStruct();

    public SparkAnalogSensorLogger() {
        super(SparkAnalogSensor.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkAnalogSensor analogSensor) {
        backend.log("", analogSensor, struct);
    }

    private static class SparkAnalogSensorStruct implements Struct<SparkAnalogSensor> {

        @Override
        public Class<SparkAnalogSensor> getTypeClass() {
            return SparkAnalogSensor.class;
        }

        @Override
        public String getTypeName() {
            return "SparkAnalogSensor";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 3;
        }

        @Override
        public String getSchema() {
            return "double position; double velocity; double voltage;";
        }

        @Override
        public SparkAnalogSensor unpack(ByteBuffer bb) {
            // Because this struct is only used in the context of serializing
            // device state for logging, returning null is fine even though it
            // technically breaks contract.
            return null;
        }

        @Override
        public void pack(ByteBuffer bb, SparkAnalogSensor value) {
            bb.putDouble(value.getPosition());
            bb.putDouble(value.getVelocity());
            bb.putDouble(value.getVoltage());
        }
    }
}
