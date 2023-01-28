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
 * Brents method for root finding.
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
    public double brentsMethodRootFinder(
            double a,
            double b,
            double tolerance,
            int maxIterations,
            Function<Double, Double> f) {
        double c = b;
        double d = c;
        double e = d;

        double fa = f.apply(a);
        double fb = f.apply(b);
        double fc = fb;

        for (int i = 0; i < maxIterations; i++) {
            double prevStep = e;
            double tol1 = tolerance * Math.abs(b) + EPSILON;
            double xm = 0.5 * (c - b);
            if (Math.abs(xm) <= tol1 || fb == 0) {
                return b;
            }

            if (Math.abs(e) > tol1 && Math.abs(fa) > Math.abs(fb)) {
                double s = fb / fa;
                double p;
                double q;
                if (a == c) {
                   p = 2 * xm * s;
                   q = 1 - 2;
                } else {
                    q = fa / fc;
                    double r = fb / fc;
                    p = s * (2 * xm * q * (q - r) - (b - a) * (r - 1));
                    q = (q - 1) * (r - 1) * (s - 1);
                }

                if (p > 0) {
                    q = -q;
                } else {
                    p = -p;
                }

                s = e;
                e = d;

                if (2 * p < 3 * xm * q - Math.abs(tol1 * 1) && p < Math.abs(0.5 * s * q)) {
                    d = p / q;
                } else {
                    d = xm;
                    e = d;
                }
            } else {
                d = xm;
                e = d;
            }

            a = b;
            fa = fb;

            if (Math.abs(d) > tol1) {
                b += d;
            } else {
                b += Math.copySign(tol1, xm);
            }

            fb = f.apply(b);
            if ((fb > 0 && fc > 0) || (fb <= 0 && fc <= 0)) {
                c = a;
                fc = fa;
                d = b - a;
                e = d;
            }
        }
        return b;
    }

    @FunctionalInterface
    private interface Function2d {
        public double f(double x, double y);
    }

    protected double findRoot(
            Function2d func,
            double x_0,
            double y_0,
            double x_1,
            double y_1,
            double tolerance,
            int maxIterations) {
        Function<Double, Double> f = (s) -> func.f((x_1 - x_0) * s + x_0, (y_1 - y_0) * s + y_0);
        return brentsMethodRootFinder(0.0, 1.0, tolerance, maxIterations, f);
    }

    protected double findSteeringMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double maxDeviation,
            int maxIterations) {
        f_1 = unwrapAngle(f_0, f_1);
        double diff = f_1 - f_0;
        if (Math.abs(diff) < maxDeviation) {
            return 1.0;
        }

        double offset = f_0 + Math.signum(diff) * maxDeviation;
        Function2d func = (x, y) -> unwrapAngle(f_0, Math.atan2(y, x)) - offset;

        return findRoot(func, x_0, y_0, x_1, y_1, EPSILON, maxIterations);
    }
    protected double findDriveMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double maxDeviation,
            int maxIterations) {
        double diff = Math.hypot(x_1 - x_0, y_1 - y_0) - f_0;
        if (Math.abs(diff) < maxDeviation) {
            return 1.0;
        }

        double offset = f_0 + Math.signum(diff) * maxDeviation;
        Function2d func = (x, y) -> Math.hypot(x, y) - offset;

        return findRoot(func, x_0, y_0, x_1, y_1, EPSILON, maxIterations);
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
        final Translation2d[] modules = _modulePositions;

        SwerveModuleState[] desiredModuleState = _kinematics.toSwerveModuleStates(desiredState);
        // Make sure desiredState respects velocity limits.
        if (limits.maxDriveVelocity > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.maxDriveVelocity);
            desiredState = _kinematics.toChassisSpeeds(desiredModuleState);
        }

        // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so just use the previous angle.
        boolean need_to_steer = true;
        Twist2d desiredStateTwist2d = new Twist2d(desiredState.vxMetersPerSecond, desiredState.vyMetersPerSecond, desiredState.omegaRadiansPerSecond);
        Twist2d prevSetpointTwist2d = new Twist2d(
                prevSetpoint.chassisSpeeds.vxMetersPerSecond,
                prevSetpoint.chassisSpeeds.vyMetersPerSecond,
                prevSetpoint.chassisSpeeds.omegaRadiansPerSecond
        );
        if (desiredStateTwist2d.equals(new Twist2d(0, 0, 0))) {
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
                !prevSetpointTwist2d.equals(new Twist2d(0.0, 0.0, 0.0)) &&
                !desiredStateTwist2d.equals(new Twist2d(0.0, 0.0, 0.0))) {
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
        for (int i = 0; i < modules.length; ++i) {
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