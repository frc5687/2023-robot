/* Team 5687 (C)2022 */
package org.frc5687.chargedup.util;

import java.util.ArrayList;
import java.util.List;
import org.frc5687.chargedup.subsystems.OutliersSubsystem;

public final class SubsystemManager {
    private final List<OutliersSubsystem> _subsystems = new ArrayList<>();
  //  private boolean _firstControlRun = true;
    private boolean _firstDataRun = true;
   // private double _controlPrevTimestamp;
    private double _dataPrevTimestamp;
   // private double _controlDt;
    private double _dataDt;
    //    private final Notifier _controlThread =
    //            new Notifier(
    //                    () -> {
    //                        synchronized (SubsystemManager.this) {
    //                            if (_firstControlRun) {
    //                                Thread.currentThread().setPriority(9);
    //                                Thread.currentThread().setName("Control Thread");
    //                                _firstControlRun = false;
    //                            }
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
    //                            if (_firstDataRun) {
    //                                Thread.currentThread().setPriority(5);
    //                                Thread.currentThread().setName("Data Thread");
    //                                _firstDataRun = false;
    //                            }
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
