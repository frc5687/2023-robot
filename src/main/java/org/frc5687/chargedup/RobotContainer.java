/* Team 5687 (C)2021 */
/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.commands.Drive;
import org.frc5687.chargedup.commands.Arm.IdleArm;
import org.frc5687.chargedup.commands.Arm.ManualDriveArm;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.commands.Elevator.ManualExtendElevator;
// import org.frc5687.chargedup.commands.EndEffector.ManualDriveGripper;
import org.frc5687.chargedup.commands.EndEffector.ManualDriveWrist;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.OutliersSubsystem;
import org.frc5687.chargedup.util.OutliersContainer;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private Pigeon2 _imu;
    private Robot _robot;
    private DriveTrain _driveTrain;
    private EndEffector _endEffector;
    private Arm _arm;
    private Elevator _elevator;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();

        // configure pigeon
        _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, "CANivore" );
        var pigeonConfig = new Pigeon2Configuration();
        _imu.getConfigurator().apply(pigeonConfig);

        _driveTrain = new DriveTrain(this, _oi, _imu);
        _elevator = new Elevator(this);
        _arm = new Arm(this);
        _endEffector = new EndEffector(this);

        _driveTrain.resetOdometry(new Pose2d(0, 0, _driveTrain.getHeading()));

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        setDefaultCommand(_elevator, new ManualExtendElevator(_elevator, _oi));
        setDefaultCommand(_endEffector, new ManualDriveWrist(_endEffector, _oi));
        setDefaultCommand(_arm, new ManualDriveArm(_arm, _oi));
        _oi.initializeButtons(_endEffector, _arm, _elevator);
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
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

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.modulePeriodic();
        } 
    }
}

