package org.team340.lib.swerve;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import org.team340.lib.logging.EpilogueProxy;

@CustomLoggerFor(SwerveAPI.class)
public class SwerveAPILogger extends ClassSpecificLogger<SwerveAPI> {

    private static final Map<SwerveAPI, Consumer<EpilogueBackend>> registry = new HashMap<>();
    private static final Function<SwerveAPI, Consumer<EpilogueBackend>> mappingFunction = api -> {
        var state = EpilogueProxy.getLogger(api.state);

        var imu = EpilogueProxy.getLogger(api.imu.getAPI());
        Map<String, Consumer<EpilogueBackend>> modules = new HashMap<>();
        for (SwerveModule module : api.modules) {
            var move = EpilogueProxy.getLogger(module.moveMotor.getAPI());
            var turn = EpilogueProxy.getLogger(module.turnMotor.getAPI());
            var encoder = EpilogueProxy.getLogger(module.encoder.getAPI());

            modules.put(module.getName(), b -> {
                move.accept(b.getNested("moveMotor"));
                turn.accept(b.getNested("turnMotor"));
                encoder.accept(b.getNested("encoder"));
            });
        }

        return backend -> {
            state.accept(backend.getNested("state"));

            var hardware = backend.getNested("hardware");
            imu.accept(hardware.getNested("imu"));
            for (var module : modules.entrySet()) {
                module.getValue().accept(hardware.getNested(module.getKey()));
            }
        };
    };

    public SwerveAPILogger() {
        super(SwerveAPI.class);
    }

    @Override
    public void update(EpilogueBackend backend, SwerveAPI api) {
        registry.computeIfAbsent(api, mappingFunction).accept(backend);
    }
}
