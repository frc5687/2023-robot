/* Team 5687 (C)2021 */
/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.chargedup.commands.Drive;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.OutliersSubsystem;
import org.frc5687.chargedup.util.OutliersContainer;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
//    private Pigeon2 _imu;
    private AHRS _imu;
    private Robot _robot;
    private DriveTrain _driveTrain;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();

        // configure pigeon
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200); // 200hz
//        _imu = new Pigeon2(RobotMap.CAN.PIDGEON.PIDGEON);
//        _imu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 10, 10);

        _driveTrain = new DriveTrain(this, _oi, _imu);
        _driveTrain.resetOdometry(new Pose2d(0, 0, _driveTrain.getHeading()));
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        startPeriodic();
    }

    public void periodic() {}

    public void disabledPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {}

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }
}

