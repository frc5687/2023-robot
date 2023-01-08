/* (C)2021 */
package org.frc5687.chargedup.util;

public interface ILoggingSource {
    void error(String message);

    void warn(String message);

    void info(String message);

    void debug(String message);
}
