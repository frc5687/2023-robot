/* Team 5687 (C)2020-2022 */
package org.frc5687.chargedup.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.lib.logging.ILoggingSource;
import org.frc5687.lib.logging.RioLogger;

public abstract class OutliersProxy implements ILoggingSource {
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

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
    }

    public abstract void updateDashboard();
}
