package org.frc5687.lib.swerve;

import java.util.function.Function;

public class SwerveSetpointGenerator {

    private final double EPSILON = 1e-15;

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

    protected double findSteeringMax(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double maxDeviation,
            int maxIterations
            ) {
        f_1 = unwrapAngle(f_0, f_1);
        double diff = f_1 - f_0;
        if (Math.abs(diff) < maxDeviation) {
            return 1.0;
        }

        double offset = f_0 + Math.signum(diff) * maxDeviation;
        Function2d func = (x, y) -> {
            return unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        };

        return findRoot(func, x_0, y_0, x_1, y_1, EPSILON, maxIterations);
    }
    }


}