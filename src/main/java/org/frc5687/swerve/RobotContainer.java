/* Team 5687 (C)2021 */
/* Team 5687 (C)2021-2022 */
package org.frc5687.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc5687.swerve.commands.Drive;
import org.frc5687.swerve.commands.OutliersCommand;
import org.frc5687.swerve.commands.TestModule;
import org.frc5687.swerve.subsystems.*;
import org.frc5687.swerve.util.*;
// import org.frc5687.lib.vision.VisionProcessor;

public class RobotContainer extends OutliersContainer {

    private OI _oi;
    // private VisionProcessor _visionProcessor;
    private Pigeon2 _imu;
    private Robot _robot;
    private DriveTrain _driveTrain;
    private TestyModule _module;
    // private PhotonProcessor _photonProcessor;
    // private Trajectories _trajectories;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        Thread.currentThread().setName("Robot Thread");
        _oi = new OI();
        // create the vision processor
        // _visionProcessor = new VisionProcessor();
        // subscribe to a vision topic for the correct data
        // _visionProcessor.createSubscriber("vision", "tcp://10.56.87.20:5557");
        // _trajectories = new Trajectories(new PathConstraints(3.0, 2.0));

//         try {
//             _photonProcessor =
// //                    new PhotonProcessor(AprilTagFieldLayout.loadFromResource("2023-chargedup.json"));
//             new PhotonProcessor(FieldConstants.aprilTags);
//         } catch (Exception e) {
//             e.getMessage();
//         }
        // configure pigeon
        _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, "CANivore");
        var pigeonConfig = new Pigeon2Configuration();
        _imu.getConfigurator().apply(pigeonConfig);

        _driveTrain = new DriveTrain(this, _imu);

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));

        // _module = new TestyModule(this, new SwerveModule(
        //     Constants.DriveTrain.SOUTH_WEST_CONFIG,
        //     RobotMap.CAN.TALONFX.SOUTH_WEST_ROTATION,
        //     RobotMap.CAN.TALONFX.SOUTH_WEST_TRANSLATION, 
        //     0, 
        //     RobotMap.CAN.CANCODER.ENCODER_SW
        // ));

        // setDefaultCommand(_module, new TestModule(_module, _oi));

        _oi.initializeButtons(_driveTrain);

        // _visionProcessor.start();
        // _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.00);
        startPeriodic();
        //        _driveTrain.plotTrajectory(TrajectoryGenerator.generateTrajectory(
        //                Constants.Auto.TrajectoryPoints.Node8.RED_NODE_EIGHT_TRAJECTORY_ONE,
        // _driveTrain.getConfig()), "one");
        //        _driveTrain.plotTrajectory(TrajectoryGenerator.generateTrajectory(
        //                Constants.Auto.TrajectoryPoints.Node8.RED_NODE_EIGHT_TRAJECTORY_TWO,
        // _driveTrain.getConfig()), "Two");
        //        _driveTrain.resetRobotPose(Constants.Auto.FieldPoses.RED_NODE_EIGHT_GOAL);
        //        _driveTrain.plotTrajectory(TrajectoryGenerator.generateTrajectory(
        //                Constants.Auto.TrajectoryPoints.Node2.RED_NODE_TWO_TRAJECTORY_ONE,
        // _driveTrain.getConfig()));
    }

    public void periodic() {
        _driveTrain.updateDashboard();
        _module.updateDashboard();
    }

    public void disabledPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {
        // _autoChooser.updateChooser();
    }

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            //_driveTrain.modulePeriodic();
        }
    }
}
