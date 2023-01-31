package org.frc5687.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.frc5687.lib.math.GeometryUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

/**
 * This is 254s SwerveSetpointGenerator but modified to use WPILibs Geometry classes and
 * an extension to the regula falsi method using Illinios or ITP for root finding.
 */
public class SwerveSetpointGenerator {

    public static class KinematicLimits {
        public double maxDriveVelocity;
        public double maxDriveAcceleration;
        public double maxSteeringVelocity;
    }
    private final SwerveDriveKinematics _kinematics;
    private final Translation2d[] _modulePositions;
    private final double EPSILON = 1e-15;

    public SwerveSetpointGenerator(final SwerveDriveKinematics kinematics, final Translation2d[] modulePositions) {
        _modulePositions = modulePositions;
        _kinematics = kinematics;
    }

    protected boolean epsilonEquals(double a, double b) {
        return (a - EPSILON <= b) && (a + EPSILON >= b);
    }

    private boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    protected double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    @FunctionalInterface
    private interface Function2d {
        public double f(double x, double y);
    }
    private double findRootIllinois(Function2d func, double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, int iterations_left) {
        if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
            return 1.0;
        }
        var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
        var x_guess = (x_1 - x_0) * s_guess + x_0;
        var y_guess = (y_1 - y_0) * s_guess + y_0;
        var f_guess = func.f(x_guess, y_guess);
        var slope_guess = (f_guess - f_0) / (s_guess - 0);
        var slope_1 = (f_1 - f_0);
        if (Math.signum(slope_guess) == Math.signum(slope_1)) {
            // guess and upper bracket have same slope, so use upper bracket.
            return s_guess + (1.0 - s_guess) * findRootIllinois(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess * findRootIllinois(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }

    private double findRootITP(Function2d func, double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, double x_2, double y_2, double f_2, int iterations_left) {
        if (iterations_left < 0 || epsilonEquals(f_1, f_1) || epsilonEquals(f_1, f_2) || epsilonEquals(f_0, f_2)) {
            return 1.0;
        }
        double a = (f_1 - f_0) / (x_1 - x_0);
        double b = (f_2 - f_1) / (x_2 - x_1);
        double c = (b - a) / (x_2 - x_1);
        double s = (a + b) / 2 - c * (x_1 - x_0) / 4;
        double x_guess = -s / c;
        double y_guess = (y_1 - y_0) * x_guess + (x_1 * y_0 - x_0 * y_1) / (x_1 - x_0);
        double f_guess = func.f(x_guess, y_guess);
        if (Math.signum(f_0) == Math.signum(f_guess)) {
            return findRootITP(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, x_2, y_2, f_2, iterations_left - 1);
        } else {
            return findRootITP(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
        }
    }

    private double findRoot(Function2d func, double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, int iterations_left) {
         return findRootIllinois(func, x_0, y_0, f_0, x_1, y_1, f_1, iterations_left);
//        return findRootITP(func, x_0, y_0, f_0, x_1, y_1, f_1, x_1, y_1, f_1, iterations_left);
    }

    protected double findSteeringMaxS(double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, double max_deviation, int max_iterations) {
        f_1 = unwrapAngle(f_0, f_1);
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_deviation) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_deviation;
        Function2d func = (x,y) -> unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    protected double findDriveMaxS(double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, double max_vel_step, int max_iterations) {
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_vel_step) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_vel_step;
        Function2d func = (x,y) -> Math.hypot(x, y) - offset;
        return  findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }


    /**
     * Generate a new setpoint.
     *
     * @param limits The kinematic limits to respect for this setpoint.
     * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous iteration setpoint instead of the actual
     *                     measured/estimated kinematic state.
     * @param desiredState The desired state of motion, such as from the driver sticks or a path following algorithm.
     * @param dt The loop time.
     * @return A Setpoint object that satisfies all of the KinematicLimits while converging to desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(final KinematicLimits limits, final SwerveSetpoint prevSetpoint, ChassisSpeeds desiredState, double dt) {
        // System.out.println("func called");
        final Translation2d[] modules = _modulePositions;

        SwerveModuleState[] desiredModuleState = _kinematics.toSwerveModuleStates(desiredState);
        // Make sure desiredState respects velocity limits.
        if (limits.maxDriveVelocity > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.maxDriveVelocity);
            desiredState = _kinematics.toChassisSpeeds(desiredModuleState);
        }

        // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so just use the previous angle.
        boolean need_to_steer = true;
        if (GeometryUtil.toTwist2d(desiredState).equals(new Twist2d(0.0, 0.0, 0.0))) {
            need_to_steer = false;
            for (int i = 0; i < modules.length; ++i) {
                desiredModuleState[i].angle = prevSetpoint.moduleStates[i].angle;
                desiredModuleState[i].speedMetersPerSecond = 0.0;
            }
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prev_vx = new double[modules.length];
        double[] prev_vy = new double[modules.length];
        Rotation2d[] prev_heading = new Rotation2d[modules.length];
        double[] desired_vx = new double[modules.length];
        double[] desired_vy = new double[modules.length];
        Rotation2d[] desired_heading = new Rotation2d[modules.length];
        boolean all_modules_should_flip = true;
        for (int i = 0; i < modules.length; ++i) {
            prev_vx[i] = prevSetpoint.moduleStates[i].angle.getCos() * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prev_vy[i] = prevSetpoint.moduleStates[i].angle.getSin() * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prev_heading[i] = prevSetpoint.moduleStates[i].angle;
            if (prevSetpoint.moduleStates[i].speedMetersPerSecond < 0.0) {
                prev_heading[i] = prev_heading[i].unaryMinus();
            }
            desired_vx[i] = desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
            desired_vy[i] = desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
            desired_heading[i] = desiredModuleState[i].angle;
            if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
                desired_heading[i] = desired_heading[i].unaryMinus();
            }
            if (all_modules_should_flip) {
                double required_rotation_rad = Math.abs(GeometryUtil.inverse(prev_heading[i]).rotateBy(desired_heading[i]).getRadians());
                if (required_rotation_rad < Math.PI / 2.0) {
                    all_modules_should_flip = false;
                }
            }
        }
        if (all_modules_should_flip &&
                !GeometryUtil.toTwist2d(prevSetpoint.chassisSpeeds).equals(new Twist2d(0.0, 0.0, 0.0)) &&
                !GeometryUtil.toTwist2d(desiredState).equals(new Twist2d(0.0, 0.0, 0.0))) {
            // It will (likely) be faster to stop the robot, rotate the modules in place to the complement of the desired
            // angle, and accelerate again.
            return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt);
        }

        // Compute the deltas between start and goal. We can then interpolate from the start state to the goal state; then
        // find the amount we can move from start towards goal in this cycle such that no kinematic limit is exceeded.
        double dx = desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds.vyMetersPerSecond;
        double dtheta = desiredState.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds.omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at desiredState.
        double min_s = 1.0;

        // In cases where an individual module is stopped, we want to remember the right steering angle to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modules.length);
        // Enforce steering velocity limits. We do this by taking the derivative of steering angle at the current angle,
        // and then backing out the maximum interpolant between start and goal states. We remember the minimum across all modules, since
        // that is the active constraint.
        final double max_theta_step = dt * limits.maxSteeringVelocity;
        for (int i = 0; i < modules.length; ++i) {
            if (!need_to_steer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates[i].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());
            if (epsilonEquals(prevSetpoint.moduleStates[i].speedMetersPerSecond, 0.0)) {
                // If module is stopped, we know that we will need to move straight to the final steering angle, so limit based
                // purely on rotation in place.
                if (epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates[i].angle));
                    continue;
                }

                var necessaryRotation = GeometryUtil.inverse(prevSetpoint.moduleStates[i].angle).rotateBy(
                        desiredModuleState[i].angle);
                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(new Rotation2d(Math.PI));
                }
                // getRadians() bounds to +/- Pi.
                final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
                    // Don't limit the global min_s;
                    continue;
                } else {
                    // Adjust steering by max_theta_step.
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates[i].angle.rotateBy(
                            Rotation2d.fromRadians(Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
                    min_s = 0.0;
                    continue;
                }
            }
            if (min_s == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            final int kMaxIterations = 8;
            double s = findSteeringMaxS(prev_vx[i], prev_vy[i], prev_heading[i].getRadians(),
                    desired_vx[i], desired_vy[i], desired_heading[i].getRadians(),
                    max_theta_step, kMaxIterations);
            min_s = Math.min(min_s, s);
        }

        // Enforce drive wheel acceleration limits.
        final double max_vel_step = dt * limits.maxDriveAcceleration;
        // System.out.println(max_vel_step);
        for (int i = 0; i < modules.length; ++i) {
            // System.out.println(min_s);

            if (min_s == 0.0) {
                // No need to carry on.
                break;
            }
            double vx_min_s = min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
            double vy_min_s = min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
            // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we already know we can't go faster
            // than that.
            // TODO(for efficiency, do all this on v^2 to save a bunch of sqrts)
            // TODO(be smarter about root finding, since this is just a quadratic in s: ((xf-x0)*s+x0)^2+((yf-y0)*s+y0)^2)
            final int kMaxIterations = 10;
            double s = min_s * findDriveMaxS(prev_vx[i], prev_vy[i], Math.hypot(prev_vx[i], prev_vy[i]),
                    vx_min_s, vy_min_s, Math.hypot(vx_min_s, vy_min_s),
                    max_vel_step, kMaxIterations);
            min_s = Math.min(min_s, s);
        }

        ChassisSpeeds retSpeeds = new ChassisSpeeds(
                prevSetpoint.chassisSpeeds.vxMetersPerSecond + min_s * dx,
                prevSetpoint.chassisSpeeds.vyMetersPerSecond + min_s * dy,
                prevSetpoint.chassisSpeeds.omegaRadiansPerSecond + min_s * dtheta);
        var retStates = _kinematics.toSwerveModuleStates(retSpeeds);
        for (int i = 0; i < modules.length; ++i) {
            final var maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (flipHeading(GeometryUtil.inverse(retStates[i].angle).rotateBy(override))) {
                    retStates[i].speedMetersPerSecond *= -1.0;
                }
                retStates[i].angle = override;
            }
            final var deltaRotation = GeometryUtil.inverse(prevSetpoint.moduleStates[i].angle).rotateBy(retStates[i].angle);
            if (flipHeading(deltaRotation)) {
                retStates[i].angle = retStates[i].angle.unaryMinus();
                retStates[i].speedMetersPerSecond *= -1.0;
            }
        }
        return new SwerveSetpoint(retSpeeds, retStates);
    }
}