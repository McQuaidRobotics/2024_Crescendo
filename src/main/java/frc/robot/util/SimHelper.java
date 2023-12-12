package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class SimHelper {

    //https://github.com/mahmoud-a-ali/scurve_traj_generation/tree/master
    //https://www.trajectorygenerator.com/ojet-online/

    // private static class StateVector {
    //     public final Double position, velocity, acceleration;

    //     public StateVector(Double position, Double velocity, Double acceleration) {
    //         this.position = position;
    //         this.velocity = velocity;
    //         this.acceleration = acceleration;
    //     }

    //     public static StateVector empty() {
    //         return new StateVector(0.0, 0.0, 0.0);
    //     }
    // }

    // private static class Boundries {
    //     public final Double maxVelocity, maxAcceleration, maxJerk;

    //     public Boundries(Double maxVelocity, Double maxAcceleration, Double maxJerk) {
    //         this.maxVelocity = maxVelocity;
    //         this.maxAcceleration = maxAcceleration;
    //         this.maxJerk = maxJerk;
    //     }

    //     public Boundries symetric() {
    //         return new Boundries(-maxVelocity, -maxAcceleration, -maxJerk);
    //     }
    // }

    // //TODO: make this able to properly represent the internal state of a motor
    // public static class SetPoint {
    //     private final Boundries upper, lower;
    //     private StateVector start_state, end_state;
    //     private Double start_time;
    //     private Boolean has_setpoint;

    //     public SetPoint(Double maxVelocity, Double maxAcceleration, Double maxJerk) {
    //         this.upper = new Boundries(maxVelocity, maxAcceleration, maxJerk);
    //         this.lower = upper.symetric();
    //         this.start_state = StateVector.empty();
    //         this.end_state = StateVector.empty();
    //         this.start_time = 0.0;
    //         this.has_setpoint = false;
    //     }

    //     public void setTarget(StateVector target) {
    //         this.end_state = target;
    //         this.has_setpoint = true;
    //         this.start_time = Timer.getFPGATimestamp();
    //     }

    //     public void setTargetVelocity(Double velocity) {
    //         this.end_state = new StateVector(null, velocity, null);
    //         this.has_setpoint = true;
    //         this.start_time = Timer.getFPGATimestamp();
    //     }

    //     public void setTargetAcceleration(Double acceleration) {
    //         this.end_state = new StateVector(null, null, acceleration);
    //         this.has_setpoint = true;
    //         this.start_time = Timer.getFPGATimestamp();
    //     }

    //     public void setTargetPosition(Double position) {
    //         this.end_state = new StateVector(position, null, null);
    //         this.has_setpoint = true;
    //         this.start_time = Timer.getFPGATimestamp();
    //     }


    //     public Double getPose() {
    //         if (!has_setpoint) {
    //             return 0.0;
    //         }
    //         return this.end_state.position;
    //         // TODO: implement
    //     }

    //     public Double getVelocity() {
    //         if (!has_setpoint) {
    //             return 0.0;
    //         }
    //         return this.end_state.velocity;
    //         // TODO: implement
    //     }
    // }

    public static class SimplePoseSim {
        //units/s
        private final Double maxVelocity;
        //units, units, seconds
        private Double startPose, endPose;
        private Double startTime;
        private Double veloStartPose, velo;
        private Boolean lastModeWasPose = true;

        public SimplePoseSim(final Double maxVelocity) {
            this.maxVelocity = maxVelocity;
        }

        public void instantSetPose(Double pose) {
            this.startPose = pose;
            this.endPose = pose;
            this.startTime = Timer.getFPGATimestamp();
            this.lastModeWasPose = true;
        }

        public Double getPose() {
            if (this.lastModeWasPose) {
                //units
                var deltaDistance = Math.abs(this.startPose - this.endPose);
                var dT = deltaDistance/this.maxVelocity;
                var timeSinceStart = Timer.getFPGATimestamp() - this.startTime;
                if (timeSinceStart > dT) {
                    return this.endPose;
                }
                var percent = timeSinceStart/dT;
                if (this.endPose < 0) {
                    return Math.min(this.startPose + percent * (this.startPose - this.endPose), this.endPose);
                } else {
                    return Math.max(this.startPose + percent * (this.endPose - this.startPose), this.endPose);
                }
            } else {
                var timeSinceStart = Timer.getFPGATimestamp() - this.startTime;
                return this.veloStartPose + timeSinceStart * this.velo;
            }
        }

        public Double getVelocity() {
            if (this.lastModeWasPose) {
                if (this.getPose() == this.endPose) {
                    return 0.0;
                }
                return this.maxVelocity;
            } else {
                return velo;
            }
        }

        public void setTargetPosition(Double pose) {
            this.startPose = this.getPose();
            this.endPose = pose;
            this.startTime = Timer.getFPGATimestamp();
            this.lastModeWasPose = true;
        }

        public void setTargetVelocity(Double velocity) {
            this.veloStartPose = this.getPose();
            this.startTime = Timer.getFPGATimestamp();
            if (Math.abs(velocity) > this.maxVelocity) {
                this.velo = Math.signum(velocity) * this.maxVelocity;
            } else {
                this.velo = velocity;
            }
            this.lastModeWasPose = false;
        }
    }
}
