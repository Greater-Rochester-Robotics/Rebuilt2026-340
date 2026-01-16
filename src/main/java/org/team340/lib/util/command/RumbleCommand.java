package org.team340.lib.util.command;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import java.util.HashMap;
import java.util.Map;

/**
 * A command that rumbles a controller while scheduled.
 */
public class RumbleCommand extends Command {

    private static final Map<GenericHID, Subsystem> controllers = new HashMap<>();

    private final GenericHID controller;
    private final RumbleType type;
    private final double value;

    /**
     * Creates a new rumble command.
     * @param controller The controller to rumble.
     * @param value The normalized value (0 to 1) to set the rumble to.
     */
    public RumbleCommand(CommandGenericHID controller, double value) {
        this(controller.getHID(), value);
    }

    /**
     * Creates a new rumble command.
     * @param controller The controller to rumble.
     * @param value The normalized value (0 to 1) to set the rumble to.
     */
    public RumbleCommand(GenericHID controller, double value) {
        this(controller, RumbleType.kBothRumble, value);
    }

    /**
     * Creates a new rumble command.
     * @param controller The controller to rumble.
     * @param type Which rumble value to set.
     * @param value The normalized value (0 to 1) to set the rumble to.
     */
    public RumbleCommand(CommandGenericHID controller, RumbleType type, double value) {
        this(controller.getHID(), type, value);
    }

    /**
     * Creates a new rumble command.
     * @param controller The controller to rumble.
     * @param type Which rumble value to set.
     * @param value The normalized value (0 to 1) to set the rumble to.
     */
    public RumbleCommand(GenericHID controller, RumbleType type, double value) {
        addRequirements(controllers.computeIfAbsent(controller, k -> new DummySubsystem()));
        this.controller = controller;
        this.type = type;
        this.value = value;
    }

    @Override
    public void execute() {
        controller.setRumble(type, value);
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(type, 0.0);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
