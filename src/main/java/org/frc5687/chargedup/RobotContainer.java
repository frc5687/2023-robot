/* Team 5687 (C)2021 */
/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.chargedup.commands.Drive;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.commands.EndEffector.ManualDriveGripper;
import org.frc5687.chargedup.commands.EndEffector.ManualDriveWrist;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.OutliersSubsystem;
import org.frc5687.chargedup.util.OutliersContainer;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private Pigeon2 _imu;
//    private AHRS _imu;
    private Robot _robot;
    private DriveTrain _driveTrain;
    private EndEffector _endEffector;
    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();

        // configure pigeon
//        _imu = new AHRS(SPI.Port.kMXP, (byte) 200); // 200hz
        _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON);

        _driveTrain = new DriveTrain(this, _oi, _imu);
        Trajectory S = TrajectoryGenerator.generateTrajectory(Constants.Auto.TrajectoryPoints.S.waypoints, _driveTrain.getConfig());
        _endEffector = new EndEffector(this);
        _driveTrain.resetOdometry(new Pose2d(0, 0, _driveTrain.getHeading()));
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        setDefaultCommand(_endEffector, new ManualDriveWrist
        (_endEffector, _oi));
        _oi.initializeButtons(_endEffector);
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

