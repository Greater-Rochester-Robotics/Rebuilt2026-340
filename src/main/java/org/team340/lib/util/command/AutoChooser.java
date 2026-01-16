package org.team340.lib.util.command;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

/**
 * Similar to {@link SendableChooser}, this class implements a selector over
 * NetworkTables for choosing a command to run during the autonomous period.
 *
 * <p>The AutoChooser binds itself to the default {@link CommandScheduler}
 * instance, and will automatically schedule the selected command at the
 * beginning of autonomous.
 */
public final class AutoChooser {

    private static final String DEFAULT = "Do Nothing";

    private final StringArrayPublisher optionsPub;
    private final StringPublisher activePub;
    private final StringSubscriber selectedSub;

    private final Map<String, Command> options = new HashMap<>();
    private final boolean renameCommands;

    private String activeName = DEFAULT;
    private Command activeCommand = Commands.none();
    private boolean enableScheduling = true;
    private boolean newSelection = false;
    private boolean running = false;

    /**
     * Creates an auto chooser, using the default {@code /Autos} table.
     */
    public AutoChooser() {
        this("/Autos", true);
    }

    /**
     * Creates an auto chooser.
     * @param name The name for the chooser in NetworkTables.
     */
    public AutoChooser(String name) {
        this(name, true);
    }

    /**
     * Creates an auto chooser.
     * @param name The name for the chooser in NetworkTables.
     * @param renameCommands If the specified names of autonomous options should
     *                       be applied to their corresponding commands when
     *                       adding them to the chooser.
     */
    public AutoChooser(String name, boolean renameCommands) {
        this.renameCommands = renameCommands;

        NetworkTable nt = NetworkTableInstance.getDefault().getTable(name);

        nt.getStringTopic(".type").publish().set("String Chooser");
        nt.getBooleanTopic(".controllable").publish().set(true);
        nt.getStringTopic("default").publish().set(DEFAULT);

        optionsPub = nt.getStringArrayTopic("options").publish();
        activePub = nt.getStringTopic("active").publish();
        selectedSub = nt.getStringTopic("selected").subscribe(DEFAULT);

        add(activeName, activeCommand);
        activePub.set(activeName);

        CommandScheduler.getInstance().getDefaultButtonLoop().bind(this::update);
    }

    /**
     * Returns the command of the currently selected auto.
     */
    public Command getSelected() {
        return activeCommand;
    }

    /**
     * Returns a {@link Trigger} that rises {@code true}
     * for one cycle after an auto is selected.
     */
    public Trigger newSelection() {
        return new Trigger(() -> newSelection);
    }

    /**
     * Returns a {@link Trigger} that is {@code true}
     * when the default option ("Do Nothing") is selected.
     */
    public Trigger defaultSelected() {
        return new Trigger(() -> activeName.equals(DEFAULT));
    }

    /**
     * Add an option to the chooser.
     * @param name The name of the option. Must be unique.
     * @param command The option's command.
     * @return A {@link Trigger} that is {@code true} when the option is selected.
     */
    public Trigger add(String name, Command command) {
        if (renameCommands) command.setName(name);

        options.put(name, command);
        optionsPub.set(options.keySet().toArray(String[]::new));
        return new Trigger(() -> activeName.equals(name));
    }

    /**
     * Enables automatically scheduling the selected command when the robot
     * is enabled in autonomous mode. This behavior is enabled by default.
     * @param enabled {@code true} to enable automatic scheduling.
     */
    public void enableScheduling(boolean enabled) {
        enableScheduling = enabled;
    }

    /**
     * Updates the auto chooser.
     */
    private void update() {
        newSelection = false;
        if (!running) {
            String selected = selectedSub.get();
            if (!selected.equals(activeName)) {
                activeName = options.containsKey(selected) ? selected : DEFAULT;
                activeCommand = options.get(activeName);
                activePub.set(activeName);
                newSelection = true;
            }
        }

        boolean schedule = DriverStation.isAutonomousEnabled() && enableScheduling;
        if (!running && schedule) {
            CommandScheduler.getInstance().schedule(activeCommand);
            running = true;
        } else if (running && !schedule) {
            activeCommand.cancel();
            running = false;
        }
    }
}
