/* (C)2021 */
package org.frc5687.swerve.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.LinkedList;
import java.util.List;

import org.frc5687.swerve.subsystems.OutliersSubsystem;

public abstract class OutliersContainer implements ILoggingSource {
    private List<OutliersSubsystem> _subsystems = new LinkedList<OutliersSubsystem>();
    private IdentityMode _identityMode;

    public OutliersContainer(IdentityMode identityMode) {
        _identityMode = identityMode;
    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
    }

    @Override
    public void error(String message) {
        RioLogger.error(this, message);
    }

    @Override
    public void warn(String message) {
        RioLogger.warn(this, message);
    }

    @Override
    public void info(String message) {
        RioLogger.info(this, message);
    }

    @Override
    public void debug(String message) {
        RioLogger.debug(this, message);
    }

    /***
     * Registers a subsystem for periodic actions.
     * @param subsystem
     */
    public void registerSubSystem(OutliersSubsystem subsystem) {
        if (!_subsystems.contains(subsystem)) {
            _subsystems.add(subsystem);
        }
    }

    /***
     * Unregisters a subsystem for periodic actions.
     * @param subsystem
     */
    public void unregisterSubSystem(OutliersSubsystem subsystem) {
        if (_subsystems.contains(subsystem)) {
            _subsystems.remove(subsystem);
        }
    }

    public void updateDashboard() {
        _subsystems.forEach((ss) -> ss.updateDashboard());
    }

    public void disabledPeriodic() {}
    ;

    public void disabledInit() {}
    ;

    public void teleopInit() {}
    ;

    public void autonomousInit() {}
    ;

    public enum IdentityMode {
        competition(0),
        practice(1),
        programming(2);

        private int _value;

        IdentityMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public IdentityMode getIdentityMode() {
        return _identityMode;
    }
}
