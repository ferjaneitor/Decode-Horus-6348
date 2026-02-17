package frc.SuperSubsystem.SuperMotors.SparkMax;

import frc.SuperSubsystem.SuperMotors.SparkMax.SparkMaxEntrys.SparkMaxMotionState;

public final class MotionProfiles {
    private MotionProfiles() {}

    public enum MotionProfileType {
        TRAPEZOIDAL,
        S_CURVE
    }

    public enum ProfileDomain {
        POSITION,
        VELOCITY
    }

    public interface MotionProfile {
        double getTotalTimeSeconds();
        SparkMaxMotionState sample(double elapsedTimeSeconds);
    }

    public static final class TrapezoidalMotionProfile implements MotionProfile {
        private final double startPosition;
        private final double endPosition;
        private final double direction;

        private final double maximumVelocity;
        private final double maximumAcceleration;

        private final double accelerationTimeSeconds;
        private final double cruiseTimeSeconds;
        private final double totalTimeSeconds;

        public TrapezoidalMotionProfile(
                double startPosition,
                double endPosition,
                double maximumVelocity,
                double maximumAcceleration
        ) {
            this.startPosition = startPosition;
            this.endPosition = endPosition;
            this.direction = Math.signum(endPosition - startPosition);

            this.maximumVelocity = maximumVelocity;
            this.maximumAcceleration = maximumAcceleration;

            Timing timing = Timing.calculate(startPosition, endPosition, maximumVelocity, maximumAcceleration);
            this.accelerationTimeSeconds = timing.accelerationTimeSeconds;
            this.cruiseTimeSeconds = timing.cruiseTimeSeconds;
            this.totalTimeSeconds = timing.totalTimeSeconds;
        }

        @Override
        public double getTotalTimeSeconds() {
            return totalTimeSeconds;
        }

        @Override
        public SparkMaxMotionState sample(double elapsedTimeSeconds) {
            if (elapsedTimeSeconds <= 0) {
                return new SparkMaxMotionState(startPosition, 0, 0, 0);
            }

            if (elapsedTimeSeconds >= totalTimeSeconds) {
                return new SparkMaxMotionState(endPosition, 0, 0, 0);
            }

            double position;
            double velocity;
            double acceleration;

            double decelerationTimeSeconds = accelerationTimeSeconds;
            double decelerationStartSeconds = accelerationTimeSeconds + cruiseTimeSeconds;

            if (elapsedTimeSeconds < accelerationTimeSeconds) {
                acceleration = maximumAcceleration;
                velocity = maximumAcceleration * elapsedTimeSeconds;
                position = startPosition + direction * 0.5 * maximumAcceleration * elapsedTimeSeconds * elapsedTimeSeconds;
            } else if (elapsedTimeSeconds < decelerationStartSeconds) {
                double timeInCruiseSeconds = elapsedTimeSeconds - accelerationTimeSeconds;

                double accelerationDistance = 0.5 * maximumAcceleration * accelerationTimeSeconds * accelerationTimeSeconds;
                double cruiseVelocity = (cruiseTimeSeconds > 0)
                        ? maximumVelocity
                        : maximumAcceleration * accelerationTimeSeconds;

                acceleration = 0.0;
                velocity = cruiseVelocity;
                position = startPosition + direction * (accelerationDistance + cruiseVelocity * timeInCruiseSeconds);
            } else {
                double timeInDecelerationSeconds = elapsedTimeSeconds - decelerationStartSeconds;

                double accelerationDistance = 0.5 * maximumAcceleration * accelerationTimeSeconds * accelerationTimeSeconds;
                double cruiseDistance = (cruiseTimeSeconds > 0) ? maximumVelocity * cruiseTimeSeconds : 0.0;

                double cruiseVelocity = (cruiseTimeSeconds > 0)
                        ? maximumVelocity
                        : maximumAcceleration * accelerationTimeSeconds;

                acceleration = -maximumAcceleration;
                velocity = cruiseVelocity - maximumAcceleration * timeInDecelerationSeconds;

                position = startPosition + direction * (
                        accelerationDistance +
                        cruiseDistance +
                        cruiseVelocity * timeInDecelerationSeconds -
                        0.5 * maximumAcceleration * timeInDecelerationSeconds * timeInDecelerationSeconds
                );

                if (timeInDecelerationSeconds > decelerationTimeSeconds) {
                    position = endPosition;
                    velocity = 0.0;
                    acceleration = 0.0;
                }
            }

            return new SparkMaxMotionState(
                    position,
                    direction * velocity,
                    direction * acceleration,
                    0.0
            );
        }

        private static final class Timing {
            private final double accelerationTimeSeconds;
            private final double cruiseTimeSeconds;
            private final double totalTimeSeconds;

            private Timing(double accelerationTimeSeconds, double cruiseTimeSeconds) {
                this.accelerationTimeSeconds = accelerationTimeSeconds;
                this.cruiseTimeSeconds = cruiseTimeSeconds;
                this.totalTimeSeconds = 2.0 * accelerationTimeSeconds + cruiseTimeSeconds;
            }

            private static Timing calculate(
                    double startPosition,
                    double endPosition,
                    double maximumVelocity,
                    double maximumAcceleration
            ) {
                double distance = Math.abs(endPosition - startPosition);

                if (maximumVelocity <= 0 || maximumAcceleration <= 0) {
                    return new Timing(0, 0);
                }

                double accelerationTimeSeconds = maximumVelocity / maximumAcceleration;
                double accelerationDistance = 0.5 * maximumAcceleration * accelerationTimeSeconds * accelerationTimeSeconds;

                double totalAccelerationDecelerationDistance = 2.0 * accelerationDistance;

                if (totalAccelerationDecelerationDistance > distance) {
                    accelerationTimeSeconds = Math.sqrt(distance / maximumAcceleration);
                    return new Timing(accelerationTimeSeconds, 0.0);
                }

                double cruiseDistance = distance - totalAccelerationDecelerationDistance;
                double cruiseTimeSeconds = cruiseDistance / maximumVelocity;
                return new Timing(accelerationTimeSeconds, cruiseTimeSeconds);
            }
        }
    }

    public static final class SCurveMotionProfile implements MotionProfile {
        private final double startPosition;
        private final double endPosition;
        private final double direction;

        private final double maximumVelocity;
        private final double maximumAcceleration;
        private final double maximumJerk;

        private final double[] phaseDurationSeconds;
        private final double[] phaseStartTimeSeconds;
        private final double[] phaseJerk;
        private final SparkMaxMotionState[] phaseStartState;

        private final double totalTimeSeconds;

        public SCurveMotionProfile(
                double startPosition,
                double endPosition,
                double maximumVelocity,
                double maximumAcceleration,
                double maximumJerk
        ) {
            this.startPosition = startPosition;
            this.endPosition = endPosition;
            this.direction = Math.signum(endPosition - startPosition);

            this.maximumVelocity = maximumVelocity;
            this.maximumAcceleration = maximumAcceleration;
            this.maximumJerk = maximumJerk;

            this.phaseDurationSeconds = new double[7];
            this.phaseStartTimeSeconds = new double[7];
            this.phaseJerk = new double[7];
            this.phaseStartState = new SparkMaxMotionState[7];

            ScurveTiming timing = ScurveTiming.calculate(
                    Math.abs(endPosition - startPosition),
                    maximumVelocity,
                    maximumAcceleration,
                    maximumJerk
            );

            phaseDurationSeconds[0] = timing.phase1TimeSeconds;
            phaseDurationSeconds[1] = timing.phase2TimeSeconds;
            phaseDurationSeconds[2] = timing.phase3TimeSeconds;
            phaseDurationSeconds[3] = timing.phase4TimeSeconds;
            phaseDurationSeconds[4] = timing.phase5TimeSeconds;
            phaseDurationSeconds[5] = timing.phase6TimeSeconds;
            phaseDurationSeconds[6] = timing.phase7TimeSeconds;

            phaseJerk[0] = +maximumJerk;
            phaseJerk[1] = 0.0;
            phaseJerk[2] = -maximumJerk;
            phaseJerk[3] = 0.0;
            phaseJerk[4] = -maximumJerk;
            phaseJerk[5] = 0.0;
            phaseJerk[6] = +maximumJerk;

            double cumulativeTimeSeconds = 0.0;
            for (int phaseIndex = 0; phaseIndex < 7; phaseIndex++) {
                phaseStartTimeSeconds[phaseIndex] = cumulativeTimeSeconds;
                cumulativeTimeSeconds += phaseDurationSeconds[phaseIndex];
            }
            this.totalTimeSeconds = cumulativeTimeSeconds;

            SparkMaxMotionState currentState = new SparkMaxMotionState(0, 0, 0, 0);
            for (int phaseIndex = 0; phaseIndex < 7; phaseIndex++) {
                phaseStartState[phaseIndex] = currentState;
                currentState = integrateConstantJerk(currentState, phaseJerk[phaseIndex], phaseDurationSeconds[phaseIndex]);
            }
        }

        @Override
        public double getTotalTimeSeconds() {
            return totalTimeSeconds;
        }

        @Override
        public SparkMaxMotionState sample(double elapsedTimeSeconds) {
            if (elapsedTimeSeconds <= 0) {
                return new SparkMaxMotionState(startPosition, 0, 0, 0);
            }

            if (elapsedTimeSeconds >= totalTimeSeconds) {
                return new SparkMaxMotionState(endPosition, 0, 0, 0);
            }

            int phaseIndex = findPhaseIndex(elapsedTimeSeconds);
            double phaseElapsedTimeSeconds = elapsedTimeSeconds - phaseStartTimeSeconds[phaseIndex];

            SparkMaxMotionState localState = integrateConstantJerk(
                    phaseStartState[phaseIndex],
                    phaseJerk[phaseIndex],
                    phaseElapsedTimeSeconds
            );

            double absolutePosition = startPosition + direction * localState.position;
            double absoluteVelocity = direction * localState.velocity;
            double absoluteAcceleration = direction * localState.acceleration;
            double absoluteJerk = direction * localState.jerk;

            return new SparkMaxMotionState(absolutePosition, absoluteVelocity, absoluteAcceleration, absoluteJerk);
        }

        private int findPhaseIndex(double elapsedTimeSeconds) {
            for (int phaseIndex = 0; phaseIndex < 7; phaseIndex++) {
                double phaseStartSeconds = phaseStartTimeSeconds[phaseIndex];
                double phaseEndSeconds = phaseStartSeconds + phaseDurationSeconds[phaseIndex];
                if (elapsedTimeSeconds >= phaseStartSeconds && elapsedTimeSeconds < phaseEndSeconds) {
                    return phaseIndex;
                }
            }
            return 6;
        }

        private static SparkMaxMotionState integrateConstantJerk(
                SparkMaxMotionState initialState,
                double constantJerk,
                double deltaTimeSeconds
        ) {
            double initialPosition = initialState.position;
            double initialVelocity = initialState.velocity;
            double initialAcceleration = initialState.acceleration;

            double acceleration = initialAcceleration + constantJerk * deltaTimeSeconds;

            double velocity = initialVelocity
                    + initialAcceleration * deltaTimeSeconds
                    + 0.5 * constantJerk * deltaTimeSeconds * deltaTimeSeconds;

            double position = initialPosition
                    + initialVelocity * deltaTimeSeconds
                    + 0.5 * initialAcceleration * deltaTimeSeconds * deltaTimeSeconds
                    + (1.0 / 6.0) * constantJerk * deltaTimeSeconds * deltaTimeSeconds * deltaTimeSeconds;

            return new SparkMaxMotionState(position, velocity, acceleration, constantJerk);
        }

        private static final class ScurveTiming {
            private final double phase1TimeSeconds;
            private final double phase2TimeSeconds;
            private final double phase3TimeSeconds;
            private final double phase4TimeSeconds;
            private final double phase5TimeSeconds;
            private final double phase6TimeSeconds;
            private final double phase7TimeSeconds;

            private ScurveTiming(
                    double phase1TimeSeconds,
                    double phase2TimeSeconds,
                    double phase3TimeSeconds,
                    double phase4TimeSeconds,
                    double phase5TimeSeconds,
                    double phase6TimeSeconds,
                    double phase7TimeSeconds
            ) {
                this.phase1TimeSeconds = phase1TimeSeconds;
                this.phase2TimeSeconds = phase2TimeSeconds;
                this.phase3TimeSeconds = phase3TimeSeconds;
                this.phase4TimeSeconds = phase4TimeSeconds;
                this.phase5TimeSeconds = phase5TimeSeconds;
                this.phase6TimeSeconds = phase6TimeSeconds;
                this.phase7TimeSeconds = phase7TimeSeconds;
            }

            private static ScurveTiming calculate(
                    double distance,
                    double maximumVelocity,
                    double maximumAcceleration,
                    double maximumJerk
            ) {
                if (maximumVelocity <= 0 || maximumAcceleration <= 0 || maximumJerk <= 0) {
                    return new ScurveTiming(0, 0, 0, 0, 0, 0, 0);
                }

                double phase1TimeSeconds = maximumAcceleration / maximumJerk;

                double velocityFromJerkPhase = 0.5 * maximumAcceleration * phase1TimeSeconds;

                double phase2TimeSeconds;
                double phase3TimeSeconds;

                if (2.0 * velocityFromJerkPhase >= maximumVelocity) {
                    phase1TimeSeconds = Math.sqrt(maximumVelocity / maximumJerk);
                    phase2TimeSeconds = 0.0;
                    phase3TimeSeconds = phase1TimeSeconds;
                } else {
                    double remainingVelocity = maximumVelocity - 2.0 * velocityFromJerkPhase;
                    phase2TimeSeconds = remainingVelocity / maximumAcceleration;
                    phase3TimeSeconds = phase1TimeSeconds;
                }

                SparkMaxMotionState stateAtStart = new SparkMaxMotionState(0, 0, 0, 0);

                SparkMaxMotionState stateAfterPhase1 = integrateConstantJerk(stateAtStart, +maximumJerk, phase1TimeSeconds);
                SparkMaxMotionState stateAfterPhase2 = integrateConstantJerk(stateAfterPhase1, 0.0, phase2TimeSeconds);
                SparkMaxMotionState stateAfterPhase3 = integrateConstantJerk(stateAfterPhase2, -maximumJerk, phase3TimeSeconds);

                double accelerationSectionDistance = stateAfterPhase3.position;
                double totalAccelerationDecelerationDistance = 2.0 * accelerationSectionDistance;

                double adjustedPhase1TimeSeconds = phase1TimeSeconds;
                double adjustedPhase2TimeSeconds = phase2TimeSeconds;
                double adjustedPhase3TimeSeconds = phase3TimeSeconds;

                double phase4TimeSeconds;

                if (totalAccelerationDecelerationDistance > distance) {
                    phase4TimeSeconds = 0.0;

                    double scaleFactor = Math.sqrt(distance / totalAccelerationDecelerationDistance);
                    adjustedPhase1TimeSeconds *= scaleFactor;
                    adjustedPhase2TimeSeconds *= scaleFactor;
                    adjustedPhase3TimeSeconds *= scaleFactor;
                } else {
                    double cruiseDistance = distance - totalAccelerationDecelerationDistance;
                    double cruiseVelocity = stateAfterPhase3.velocity;
                    phase4TimeSeconds = (cruiseVelocity > 1e-9) ? cruiseDistance / cruiseVelocity : 0.0;
                }

                double phase5TimeSeconds = adjustedPhase3TimeSeconds;
                double phase6TimeSeconds = adjustedPhase2TimeSeconds;
                double phase7TimeSeconds = adjustedPhase1TimeSeconds;

                return new ScurveTiming(
                        adjustedPhase1TimeSeconds,
                        adjustedPhase2TimeSeconds,
                        adjustedPhase3TimeSeconds,
                        phase4TimeSeconds,
                        phase5TimeSeconds,
                        phase6TimeSeconds,
                        phase7TimeSeconds
                );
            }
        }
    }
}
