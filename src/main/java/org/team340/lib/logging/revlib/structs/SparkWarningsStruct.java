package org.team340.lib.logging.revlib.structs;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class SparkWarningsStruct implements Struct<Integer> {

    public static final SparkWarningsStruct struct = new SparkWarningsStruct();

    @Override
    public Class<Integer> getTypeClass() {
        return Integer.class;
    }

    @Override
    public String getTypeName() {
        return "SparkWarnings";
    }

    @Override
    public int getSize() {
        return kSizeInt8;
    }

    @Override
    public String getSchema() {
        return (
            "bool brownout:1; "
            + "bool overcurrent:1; "
            + "bool escEeprom:1; "
            + "bool extEeprom:1; "
            + "bool sensor:1; "
            + "bool stall:1; "
            + "bool hasReset:1; "
            + "bool other:1;"
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
