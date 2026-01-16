package org.team340.lib.logging.revlib.structs;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class SparkFaultsStruct implements Struct<Integer> {

    public static final SparkFaultsStruct struct = new SparkFaultsStruct();

    @Override
    public Class<Integer> getTypeClass() {
        return Integer.class;
    }

    @Override
    public String getTypeName() {
        return "SparkFaults";
    }

    @Override
    public int getSize() {
        return kSizeInt8;
    }

    @Override
    public String getSchema() {
        return (
            "bool other:1; "
            + "bool motorType:1; "
            + "bool sensor:1; "
            + "bool can:1; "
            + "bool temperature:1; "
            + "bool gateDriver:1; "
            + "bool escEeprom:1; "
            + "bool firmware:1;"
        );
    }

    @Override
    public Integer unpack(ByteBuffer bb) {
        return (int) bb.get() & 0xff;
    }

    @Override
    public void pack(ByteBuffer bb, Integer value) {
        bb.put(value.byteValue());
    }

    @Override
    public boolean isImmutable() {
        return true;
    }
}
