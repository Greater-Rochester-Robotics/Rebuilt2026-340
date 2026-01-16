package org.team340.lib.logging.revlib.structs;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public abstract class SparkBaseStruct<T extends SparkBase> implements Struct<T> {

    @Override
    public int getSize() {
        return kSizeDouble * 7 + SparkFaultsStruct.struct.getSize() * 2 + SparkWarningsStruct.struct.getSize() * 2;
    }

    @Override
    public String getSchema() {
        return (
            "double appliedOutput; "
            + "double appliedVoltage; "
            + "double closedLoopSetpoint; "
            + "double motorTemperature; "
            + "double outputCurrent; "
            + "double position; "
            + "double velocity; "
            + "SparkFaults faults; "
            + "SparkFaults stickyFaults; "
            + "SparkWarnings warnings; "
            + "SparkWarnings stickyWarnings;"
        );
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] { SparkFaultsStruct.struct, SparkWarningsStruct.struct };
    }

    @Override
    public T unpack(ByteBuffer bb) {
        // Because this struct is only used in the context of serializing
        // device state for logging, returning null is fine even though it
        // technically breaks contract.
        return null;
    }

    @Override
    public void pack(ByteBuffer bb, T value) {
        double appliedOutput = value.getAppliedOutput();
        var closedLoopController = value.getClosedLoopController();
        var encoder = value.getEncoder();

        bb.putDouble(appliedOutput);
        bb.putDouble(appliedOutput * value.getBusVoltage());
        bb.putDouble(closedLoopController.getSetpoint());
        bb.putDouble(value.getMotorTemperature());
        bb.putDouble(value.getOutputCurrent());
        bb.putDouble(encoder.getPosition());
        bb.putDouble(encoder.getVelocity());

        SparkFaultsStruct.struct.pack(bb, value.getFaults().rawBits);
        SparkFaultsStruct.struct.pack(bb, value.getStickyFaults().rawBits);
        SparkWarningsStruct.struct.pack(bb, value.getWarnings().rawBits);
        SparkWarningsStruct.struct.pack(bb, value.getStickyWarnings().rawBits);
    }
}
