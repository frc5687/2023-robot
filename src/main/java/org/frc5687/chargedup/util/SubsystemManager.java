/* Team 5687 (C)2022 */
package org.frc5687.chargedup.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.OutliersSubsystem;

import java.util.ArrayList;
import java.util.List;

public final class SubsystemManager {
    private final List<OutliersSubsystem> _subsystems = new ArrayList<>();
    private double _controlPrevTimestamp;
    private double _dataPrevTimestamp;
    private double _controlDt;
    private double _dataDt;
//    private final Notifier _controlThread =
//            new Notifier(
//                    () -> {
//                        synchronized (SubsystemManager.this) {
//                            final double timestamp = Timer.getFPGATimestamp();
//                            _controlDt = timestamp - _controlPrevTimestamp;
//                            _controlPrevTimestamp = timestamp;
//                            _subsystems.forEach(p -> p.controlPeriodic(timestamp));
//                        }
//                    });
//    private final Notifier _dataThread =
//            new Notifier(
//                    () -> {
//                        synchronized (SubsystemManager.this) {
//                            final double timestamp = Timer.getFPGATimestamp();
//                            _dataDt = timestamp - _dataPrevTimestamp;
//                            _dataPrevTimestamp = timestamp;
//                            _subsystems.forEach(p -> p.dataPeriodic(timestamp));
//                        }
//                    });

    public SubsystemManager() {}

    public void addSubsystem(OutliersSubsystem system) {
        _subsystems.add(system);
    }

    public void removeSubsystem(OutliersSubsystem system) {
        _subsystems.add(system);
    }

    public void startPeriodic() {
//        _controlThread.startPeriodic(Constants.CONTROL_PERIOD);
//        _dataThread.startPeriodic(Constants.DATA_PERIOD);
    }

    public void stopPeriodic() {
//        _controlThread.stop();
//        _dataThread.stop();
    }

    public void updateDashboard() {
        _subsystems.forEach(OutliersSubsystem::updateDashboard);
        outputToDashboard();
    }

    public void outputToDashboard() {
//        SmartDashboard.putNumber("Periodic Control DT", _controlDt);
//        SmartDashboard.putNumber("Periodic Data DT", _dataDt);
    }
}
