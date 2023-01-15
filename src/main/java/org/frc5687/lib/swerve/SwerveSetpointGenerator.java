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

    protected double findSteeringMaxS(
            double x0, double y0,
            double f0,
            double x1, double y1,
            double f1,
            double max_deviation, int max_iterations) {
        f1 = unwrapAngle(f0, f1);
        double tolerance = 0.0001;
        double diff = f1 = f0;

        if (Math.abs(diff) <= max_deviation) {
            return 1.0;
        }

        double s = brentsMethodRootFinder(0, 1, tolerance, max_iterations, s -> {
            double x = x0 + s * (x1 - x0);
            double y = y0 + s * (y1 - y0);
            double theta = unwrapAngle(f0, Math.atan2(y, x));
            double maxThetaStep = 
                }
                )
    }


}