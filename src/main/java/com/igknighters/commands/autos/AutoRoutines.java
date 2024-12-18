package com.igknighters.commands.autos;

import com.igknighters.subsystems.SubsystemResources.AllSubsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.igknighters.commands.autos.Waypoints.*;

import com.igknighters.Localizer;
import com.igknighters.Robot;

public class AutoRoutines extends AutoCommands {

    private final boolean disabled;

    public AutoRoutines(AllSubsystems allSubsystems, Localizer localizer) {
        super(allSubsystems.swerve.get(), allSubsystems.stem.get(), allSubsystems.umbrella.get(), allSubsystems.vision.get(), localizer);
        disabled = !allSubsystems.hasAllSubsystems();

        if (Robot.isSimulation()) {
            new Trigger(DriverStation::isAutonomousEnabled).onTrue(
                Commands.waitSeconds(15.3)
                .andThen(() -> DriverStationSim.setEnabled(false))
                .withName("Simulated Auto Ender")
            );
        }
    }

    private AutoRoutine disabledAuto(AutoFactory factory) {
        return factory.commandAsAutoRoutine(Commands.print("AutoRoutines disabled"));
    }

    private Command resetOdometry(AutoTrajectory traj, AutoRoutine routine) {
        var optPose = traj.getInitialPose();
        if (optPose.isEmpty()) {
            routine.kill();
            return Commands.print("Killed loop due to lack of startung pose");
        }
        var pose = optPose.get();
        return loggedCmd(swerve.runOnce(() -> {
            localizer.reset(pose);
            swerve.setYaw(pose.getRotation());
        }).withName("ResetOdometry"));
    }



    /**
     * Shoots starting note,
     * Tries to pickup C1 and shoots it if we have it,
     * Picks up M1 and shoots it if we have it,
     * If we don't have M1, picks up M2 and shoots it if we have it,
     * If we don't have M2, picks up M3 and shoots it if we have it,
     * After we shoot the second note, we pickup C2 and shoot it if we have it.
     * Finally we pickup C3 and shoot it.
     * 
     * @param factory The factory to create trajectories with
     * @return The {@link AutoRoutine}
     */
    public AutoRoutine fivePieceAmpSide(final AutoFactory factory) {
        if (disabled) return disabledAuto(factory);

        final AutoRoutine routine = factory.newRoutine("FivePieceAmpSide");

        final AutoTrajectory ampToC1 = factory.trajectory(AMP.to(C1), routine);
        final AutoTrajectory c1ToM1 = factory.trajectory(C1.to(M1), routine);
        final AutoTrajectory m1ToM2 = factory.trajectory(M1.to(M2), routine);
        final AutoTrajectory m2ToM3 = factory.trajectory(M2.to(M3), routine);
        final AutoTrajectory m1ToS1 = factory.trajectory(M1.to(S1), routine);
        final AutoTrajectory m2ToS1 = factory.trajectory(M2.to(S1), routine);
        final AutoTrajectory m3ToS2 = factory.trajectory(M3.to(S2), routine);
        final AutoTrajectory s1ToC2 = factory.trajectory(S1.to(C2), routine);
        final AutoTrajectory s2ToC2 = factory.trajectory(S2.to(C2), routine);
        final AutoTrajectory c2ToC3 = factory.trajectory(C2.to(C3), routine);

        // entry point for the auto
        routine.enabled().onTrue(
            resetOdometry(ampToC1, routine).andThen(
                autoShootBegining(),
                Commands.parallel(
                    intakeGamepieceNoStow(),
                    Commands.waitSeconds(0.2)
                        .andThen(new ScheduleCommand(ampToC1.cmd()))
                )
            ).withName("FivePieceAmpSideEntry")
        );

        // picking up first note and shooting if we have a note
        ampToC1.active().onTrue(intakeGamepieceNoStow());
        ampToC1.done().onTrue(autoShootThenTraj(routine, c1ToM1));

        // picking up the second note and branching based on whether we're holding a gamepiece
        c1ToM1.atTime(0.35).onTrue(intakeGamepieceNoStow());
        c1ToM1.done().and(yeGp(routine)).onTrue(m1ToS1.cmd());
        c1ToM1.done().and(noGp(routine)).onTrue(m1ToM2.cmd());

        // the branch where we're holding a gamepiece
        m1ToS1.active().onTrue(aimStem(m1ToS1));
        m1ToS1.done().onTrue(autoShootThenTraj(routine, s1ToC2));

        // the branch where we're not holding a gamepiece
        m1ToM2.active().onTrue(intakeGamepieceNoStow());
        m1ToM2.done().and(yeGp(routine)).onTrue(m2ToS1.cmd());
        m1ToM2.done().and(noGp(routine)).onTrue(m2ToM3.cmd());

        // the branch where we're holding a gamepiece
        m2ToS1.active().onTrue(aimStem(m2ToS1));
        m2ToS1.done().onTrue(autoShootThenTraj(routine, s1ToC2));

        // the branch where we're not holding a gamepiece
        m2ToM3.active().onTrue(intakeGamepieceNoStow());
        m1ToM2.done().and(yeGp(routine)).onTrue(m3ToS2.cmd());
        m1ToM2.done().and(noGp(routine)).onTrue(stow());

        m3ToS2.active().onTrue(stow());
        m3ToS2.done().onTrue(autoShootThenTraj(routine, s2ToC2));

        // picking up the third note, only shooting if we have a note
        s1ToC2.active().or(s2ToC2.active()).onTrue(intakeGamepieceNoStow());
        s1ToC2.done().or(s2ToC2.done()).onTrue(autoShootThenTraj(routine, c2ToC3));

        // picking up the fourth note and shooting if we have a note
        c2ToC3.active().onTrue(intakeGamepieceNoStow());
        c2ToC3.done().onTrue(autoShoot(routine));

        return routine;
    }

    /**
     * 
     * 
     * @param factory The factory to create trajectories with
     * @return The {@link AutoRoutine}
     */
    public AutoRoutine sixPieceFarAmpSide(AutoFactory factory) {
        if (disabled) return disabledAuto(factory);

        final AutoRoutine routine = factory.newRoutine("SixPieceFarAmpSide");

        final AutoTrajectory ampToC1 = factory.trajectory(AMP.to(C1), routine);
        final AutoTrajectory c1ToM1 = factory.trajectory(C1.to(M1), routine);
        final AutoTrajectory m1ToM2 = factory.trajectory(M1.to(M2), routine);
        final AutoTrajectory m2ToM3 = factory.trajectory(M2.to(M3), routine);
        final AutoTrajectory m1ToS1 = factory.trajectory(M1.to(S1), routine);
        final AutoTrajectory m2ToS1 = factory.trajectory(M2.to(S1), routine);
        final AutoTrajectory m3ToS2 = factory.trajectory(M3.to(S2), routine);
        final AutoTrajectory s1ToC2 = factory.trajectory(S1.to(C2), routine);
        final AutoTrajectory s2ToC2 = factory.trajectory(S2.to(C2), routine);
        final AutoTrajectory s1ToM2 = factory.trajectory(S1.to(M2), routine);
        final AutoTrajectory s1ToM3 = factory.trajectory(S1.to(M3), routine);
        final AutoTrajectory c2ToSUB = factory.trajectory(C2.to(SUB), routine);


        // entry point for the auto
        routine.enabled().onTrue(
            resetOdometry(ampToC1, routine).andThen(
                autoShootBegining(),
                Commands.parallel(
                    intakeGamepieceNoStow(),
                    Commands.waitSeconds(0.2)
                        .andThen(new ScheduleCommand(ampToC1.cmd()))
                )
            ).withName("SixPieceAmpSideEntry")
        );

        // picking up first note and shooting if we have a note
        ampToC1.active().onTrue(intakeGamepieceNoStow());
        ampToC1.done().onTrue(autoShootThenTraj(routine, c1ToM1));

        // picking up the second note and branching based on whether we're holding a gamepiece
        c1ToM1.atTime(0.35).onTrue(intakeGamepieceNoStow());
        c1ToM1.done().and(yeGp(routine)).onTrue(m1ToS1.cmd());
        c1ToM1.done().and(noGp(routine)).onTrue(m1ToM2.cmd());

        // the branch where we're holding a gamepiece
        m1ToS1.active().onTrue(aimStem(m1ToS1));
        m1ToS1.done().onTrue(autoShootThenTraj(routine, s1ToM2));

        // the branch where we're not holding a gamepiece
        m1ToM2.active().onTrue(intakeGamepieceNoStow());
        m1ToM2.done().and(yeGp(routine)).onTrue(m2ToS1.cmd());
        m1ToM2.done().and(noGp(routine)).onTrue(m2ToM3.cmd());

        // the branch where we're holding a gamepiece
        m2ToS1.active().onTrue(aimStem(m2ToS1));
        m2ToS1.done().onTrue(autoShootThenTraj(routine, s1ToM3));

        // the branch where we're not holding a gamepiece
        m2ToM3.active().onTrue(intakeGamepieceNoStow());
        m2ToM3.done().and(yeGp(routine)).onTrue(m3ToS2.cmd());
        m2ToM3.done().and(noGp(routine)).onTrue(stow());

        m3ToS2.active().onTrue(stow());
        m3ToS2.done().onTrue(autoShootThenTraj(routine, s2ToC2));

        s1ToM2.active().onTrue(intakeGamepieceNoStow());
        s1ToM2.done().and(yeGp(routine)).onTrue(m2ToS1.cmd());
        s1ToM2.done().and(noGp(routine)).onTrue(m2ToM3.cmd());

        s1ToM3.active().onTrue(intakeGamepieceNoStow());
        s1ToM3.done().and(yeGp(routine)).onTrue(m3ToS2.cmd());
        s1ToM3.done().and(noGp(routine)).onTrue(stow());

        // picking up the third note, only shooting if we have a note
        s1ToC2.active().or(s2ToC2.active()).onTrue(intakeGamepieceNoStow());
        s1ToC2.done().or(s2ToC2.done()).and(yeGp(routine)).onTrue(
            c2ToSUB.cmd()
        );

        // shoot from the subwoofer
        c2ToSUB.active().onTrue(aimSub());
        c2ToSUB.done().onTrue(aimSub().andThen(feedShooter()).withName("AimSubThenFeed"));

        return routine;
    }

    /**
     * 
     * 
     * @param factory The factory to create trajectories with
     * @return The {@link AutoRoutine}
     */
    public AutoRoutine celtxAuto(AutoFactory factory) {
        if (disabled) return disabledAuto(factory);

        final AutoRoutine routine = factory.newRoutine("SixPieceFarAmpSide");

        final AutoTrajectory ampToC1 = factory.trajectory(AMP.to(C1), routine);
        final AutoTrajectory c1ToM1 = factory.trajectory(C1.to(M1), routine);
        final AutoTrajectory m1ToM2 = factory.trajectory(M1.to(M2), routine);
        final AutoTrajectory m1ToS1 = factory.trajectory(M1.to(S1), routine);
        final AutoTrajectory m2ToS1 = factory.trajectory(M2.to(S1), routine);
        final AutoTrajectory s1ToM2 = factory.trajectory(S1.to(M2), routine);
        final AutoTrajectory s1ToM3 = factory.trajectory(S1.to(M3), routine);


        // entry point for the auto
        routine.enabled().onTrue(
            resetOdometry(ampToC1, routine).andThen(
                autoShootBegining(),
                Commands.race(
                    intakeGamepieceNoStow(),
                    Commands.waitSeconds(0.45)
                        .andThen(ampToC1.cmd())
                )
            ).withName("SixPieceAmpSideEntry")
        );

        // picking up first note and shooting if we have a note
        ampToC1.done().onTrue(autoShootThenTraj(routine, c1ToM1));

        // picking up the second note and branching based on whether we're holding a gamepiece
        c1ToM1.atTime(0.35).onTrue(intakeGamepieceNoStow());
        c1ToM1.done().and(yeGp(routine)).onTrue(m1ToS1.cmd());
        c1ToM1.done().and(noGp(routine)).onTrue(m1ToM2.cmd());

        // the branch where we're holding a gamepiece
        m1ToS1.active().onTrue(aimStem(m1ToS1));
        m1ToS1.done().onTrue(autoShootThenTraj(routine, s1ToM2));

        // the branch where we're not holding a gamepiece
        m1ToM2.active().onTrue(intakeGamepieceNoStow());
        m1ToM2.done().onTrue(m2ToS1.cmd());

        // the branch where we're holding a gamepiece
        m2ToS1.active().onTrue(aimStem(m2ToS1));
        m2ToS1.done().onTrue(autoShootThenTraj(routine, s1ToM3));

        s1ToM2.active().onTrue(intakeGamepieceNoStow());
        s1ToM2.done().onTrue(m2ToS1.cmd());

        s1ToM3.active().onTrue(stow());

        return routine;
    }

    /**
     * 
     * 
     * 
     * @param factory The factory to create trajectories with
     * @return The {@link AutoRoutine}
     */
    public AutoRoutine fourPieceSourceSide(final AutoFactory factory) {
        if (disabled) return disabledAuto(factory);

        final AutoRoutine routine = factory.newRoutine("FourPieceSourceSide");

        final AutoTrajectory srcToM5 = factory.trajectory(SRC.to(M5), routine);
        final AutoTrajectory m5ToM4 = factory.trajectory(M5.to(M4), routine);
        final AutoTrajectory m5ToS3 = factory.trajectory(M5.to(S3), routine);
        final AutoTrajectory m4ToS3 = factory.trajectory(M4.to(S3), routine);
        final AutoTrajectory s3ToM4 = factory.trajectory(S3.to(M4), routine);
        final AutoTrajectory s3ToM3 = factory.trajectory(S3.to(M3), routine);
        final AutoTrajectory m3ToS2 = factory.trajectory(M3.to(S2), routine);


        // entry point for the auto
        routine.enabled().onTrue(
            resetOdometry(srcToM5, routine).andThen(
                autoShootBegining(),
                Commands.parallel(
                    intakeGamepieceNoStow(),
                    srcToM5.cmd()
                )
            ).withName("FourPieceSourceSideEntry")
        );

        srcToM5.done().and(yeGp(routine)).onTrue(m5ToS3.cmd());
        srcToM5.done().and(noGp(routine)).onTrue(m5ToM4.cmd());

        m5ToS3.active().onTrue(aimStem(m5ToS3));
        m5ToS3.done().onTrue(autoShootThenTraj(routine, s3ToM4));

        s3ToM4.active().onTrue(intakeGamepieceNoStow());
        s3ToM4.done().and(yeGp(routine)).onTrue(m4ToS3.cmd());
        s3ToM4.done().and(noGp(routine)).onTrue(stow());

        m4ToS3.active().onTrue(aimStem(m4ToS3));
        m4ToS3.done().onTrue(autoShootThenTraj(routine, s3ToM3));

        s3ToM3.active().onTrue(intakeGamepieceNoStow());
        s3ToM3.done().and(yeGp(routine)).onTrue(m3ToS2.cmd());
        s3ToM3.done().and(noGp(routine)).onTrue(stow());

        m3ToS2.active().onTrue(aimStem(m3ToS2));
        m3ToS2.done().and(yeGp(routine)).onTrue(autoShoot(routine));
        m3ToS2.done().and(noGp(routine)).onTrue(stow());

        return routine;
    }

    public AutoRoutine justFeed(final AutoFactory factory) {
        return factory.commandAsAutoRoutine(feedShooter());
    }

    // /**
    //  * 
    //  * 
    //  * 
    //  * @param factory The factory to create trajectories with
    //  * @return The command that represents the auto routine
    //  */
    // public Command threePieceSubMiddle(AutoFactory factory) {
    //     if (disabled) return disabledAuto();

    //     final AutoRoutine loop = factory.newRoutine("ThreePieceSubMiddle");

    //     final AutoTrajectory subToC3 = factory.trajectory(SUB.to(C3), loop);
    //     final AutoTrajectory c3ToM3 = factory.trajectory(C3.to(M3), loop);
    //     final AutoTrajectory m3ToS2 = factory.trajectory(M3.to(S2), loop);
    //     final AutoTrajectory s2ToM3 = factory.trajectory(S2.to(M3), loop);

    //     // entry point for the auto
    //     loop.enabled().onTrue(
    //         resetOdometry(subToC3, loop).andThen(
    //             Commands.parallel(
    //                 UmbrellaCommands.waitUntilSpunUp(umbrella, 5000, 0.1),
    //                 aimSub()
    //             ),
    //             feedShooter(),
    //             Commands.parallel(
    //                 intakeGamepieceNoStow(),
    //                 Commands.waitSeconds(0.2)
    //                     .andThen(subToC3.cmd())
    //             )
    //         ).withName("ThreePieceSubMiddleEntry")
    //     );

    //     subToC3.done().onTrueWith(
    //         yeGp(loop),
    //         autoShoot(),
    //         c3ToM3.cmd()
    //     );

    //     c3ToM3.atTime(1.5).onTrue(intakeGamepieceNoStow());
    //     c3ToM3.done().onTrueWith(
    //         yeGp(loop),
    //         m3ToS2.cmd(),
    //         stow()
    //     );


    //     m3ToS2.active().onTrue(aimStem(m3ToS2));
    //     m3ToS2.done().onTrueWith(
    //         yeGp(loop),
    //         autoShoot(),
    //         s2ToM3.cmd()
    //     );

    //     s2ToM3.active().onTrue(stow());

    //     return loop.cmd().withName("FourPieceSourceSide");
    // }

    // public Command driveForward(AutoFactory factory) {
    //     if (disabled) return disabledAuto();

    //     final AutoRoutine loop = factory.newRoutine("DriveForward");

    //     final ChoreoAutoTrajectory ampToC1 = factory.traj(AMP.to(C1), loop);
    //     final ChoreoAutoTrajectory c1ToM1 = factory.traj(C1.to(M1), loop);
    //     final ChoreoAutoTrajectory m1ToM2 = factory.traj(M1.to(M2), loop);
    //     final ChoreoAutoTrajectory m2ToM3 = factory.traj(M2.to(M3), loop);
    //     final ChoreoAutoTrajectory m1ToS1 = factory.traj(M1.to(S1), loop);
    //     final ChoreoAutoTrajectory m2ToS1 = factory.traj(M2.to(S1), loop);
    //     final ChoreoAutoTrajectory m3ToS2 = factory.traj(M3.to(S2), loop);
    //     final ChoreoAutoTrajectory s1ToC2 = factory.traj(S1.to(C2), loop);
    //     final ChoreoAutoTrajectory s2ToC2 = factory.traj(S2.to(C2), loop);
    //     final ChoreoAutoTrajectory s1ToM2 = factory.traj(S1.to(M2), loop);
    //     final ChoreoAutoTrajectory s1ToM3 = factory.traj(S1.to(M3), loop);
    //     final ChoreoAutoTrajectory c2ToSUB = factory.traj(C2.to(SUB), loop);


    //     // entry point for the auto
    //     loop.enabled().onTrue(
    //         resetOdometry(ampToC1).andThen(
    //             autoShootBegining(),
    //             Commands.race(
    //                 intakeGamepieceNoStow(),
    //                 Commands.waitSeconds(0.2)
    //                     .andThen(ampToC1.cmd())
    //             )
    //         ).withName("SixPieceAmpSideEntry")
    //     );

    //     // picking up first note and shooting if we have a note
    //     ampToC1.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         c1ToM1.cmd()
    //     );

    //     // picking up the second note and branching based on whether we're holding a gamepiece
    //     c1ToM1.atTime(0.35).onTrue(intakeGamepieceNoStow());
    //     c1ToM1.done().onTrueWith(
    //         gamepiece(loop),
    //         m1ToS1.cmd(),
    //         m1ToM2.cmd()
    //     );

    //     // the branch where we're holding a gamepiece
    //     m1ToS1.active().onTrue(aimStem(m1ToS1));
    //     m1ToS1.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         s1ToM2.cmd()
    //     );

    //     // the branch where we're not holding a gamepiece
    //     m1ToM2.active().onTrue(intakeGamepieceNoStow());
    //     m1ToM2.done().onTrueWith(
    //         gamepiece(loop),
    //         m2ToS1.cmd(),
    //         m2ToM3.cmd()
    //     );

    //     // the branch where we're holding a gamepiece
    //     m2ToS1.active().onTrue(aimStem(m2ToS1));
    //     m2ToS1.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         s1ToM3.cmd()
    //     );

    //     // the branch where we're not holding a gamepiece
    //     m2ToM3.active().onTrue(intakeGamepieceNoStow());
    //     m2ToM3.done().onTrueWith(
    //         gamepiece(loop),
    //         m3ToS2.cmd(),
    //         stow()
    //     );

    //     m3ToS2.active().onTrue(stow());
    //     m3ToS2.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         s2ToC2.cmd()
    //     );

    //     s1ToM2.active().onTrue(intakeGamepieceNoStow());
    //     s1ToM2.done().onTrueWith(
    //         gamepiece(loop),
    //         m2ToS1.cmd(),
    //         m2ToM3.cmd()
    //     );

    //     s1ToM3.active().onTrue(intakeGamepieceNoStow());
    //     s1ToM3.done().onTrueWith(
    //         gamepiece(loop),
    //         m3ToS2.cmd(),
    //         stow()
    //     );

    //     // picking up the third note, only shooting if we have a note
    //     s1ToC2.active().or(s2ToC2.active()).onTrue(intakeGamepieceNoStow());
    //     s1ToC2.done().or(s2ToC2.done()).and(gamepiece(loop)).onTrue(
    //         c2ToSUB.cmd()
    //     );

    //     // shoot from the subwoofer
    //     c2ToSUB.active().onTrue(aimSub());
    //     c2ToSUB.done().onTrue(aimSub().andThen(feedShooter()).withName("AimSubThenFeed"));

    //     return loop.cmd().withName("SixPieceFarAmpSide");
    // }

    // /**
    //  * 
    //  * 
    //  * 
    //  * @param factory The factory to create trajectories with
    //  * @return The command that represents the auto routine
    //  */
    // public Command fourPieceSourceSide(ChoreoAutoFactory factory) {
    //     if (disabled) return disabledAuto();

    //     final ChoreoAutoRoutine loop = factory.newRoutine();

    //     final ChoreoAutoTrajectory srcToM5 = factory.traj(SRC.to(M5), loop);
    //     final ChoreoAutoTrajectory m5ToM4 = factory.traj(M5.to(M4), loop);
    //     final ChoreoAutoTrajectory m5ToS3 = factory.traj(M5.to(S3), loop);
    //     final ChoreoAutoTrajectory m4ToS3 = factory.traj(M4.to(S3), loop);
    //     final ChoreoAutoTrajectory s3ToM4 = factory.traj(S3.to(M4), loop);
    //     final ChoreoAutoTrajectory s3ToM3 = factory.traj(S3.to(M3), loop);
    //     final ChoreoAutoTrajectory m3ToS2 = factory.traj(M3.to(S2), loop);


    //     // entry point for the auto
    //     loop.enabled().onTrue(
    //         resetOdometry(srcToM5).andThen(
    //             autoShootBegining(),
    //             Commands.parallel(
    //                 intakeGamepieceNoStow(),
    //                 srcToM5.cmd()
    //             )
    //         ).withName("FourPieceSourceSideEntry")
    //     );

    //     // picking up first note and shooting if we have a note
    //     srcToM5.done().onTrueWith(
    //         gamepiece(loop),
    //         m5ToS3.cmd(),
    //         m5ToM4.cmd()
    //     );

    //     m5ToS3.active().onTrue(aimStem(m5ToS3));
    //     m5ToS3.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         s3ToM4.cmd()
    //     );

    //     s3ToM4.active().onTrue(intakeGamepieceNoStow());
    //     s3ToM4.done().onTrueWith(
    //         gamepiece(loop),
    //         m4ToS3.cmd(),
    //         stow()
    //     );

    //     m4ToS3.active().onTrue(aimStem(m4ToS3));
    //     m4ToS3.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         s3ToM3.cmd()
    //     );

    //     s3ToM3.active().onTrue(intakeGamepieceNoStow());
    //     s3ToM3.done().onTrueWith(
    //         gamepiece(loop),
    //         m3ToS2.cmd(),
    //         stow()
    //     );

    //     m3ToS2.active().onTrue(aimStem(m3ToS2));
    //     m3ToS2.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         stow()
    //     );

    //     return loop.cmd().withName("FourPieceSourceSide");
    // }

    // /**
    //  * 
    //  * 
    //  * 
    //  * @param factory The factory to create trajectories with
    //  * @return The command that represents the auto routine
    //  */
    // public Command threePieceSubMiddle(ChoreoAutoFactory factory) {
    //     if (disabled) return disabledAuto();

    //     final ChoreoAutoRoutine loop = factory.newRoutine();

    //     final ChoreoAutoTrajectory subToC3 = factory.traj(SUB.to(C3), loop);
    //     final ChoreoAutoTrajectory c3ToM3 = factory.traj(C3.to(M3), loop);
    //     final ChoreoAutoTrajectory m3ToS2 = factory.traj(M3.to(S2), loop);
    //     final ChoreoAutoTrajectory s2ToM3 = factory.traj(S2.to(M3), loop);

    //     // entry point for the auto
    //     loop.enabled().onTrue(
    //         resetOdometry(subToC3).andThen(
    //             Commands.parallel(
    //                 UmbrellaCommands.waitUntilSpunUp(umbrella, 5000, 0.1),
    //                 aimSub()
    //             ),
    //             feedShooter(),
    //             Commands.parallel(
    //                 intakeGamepieceNoStow(),
    //                 Commands.waitSeconds(0.2)
    //                     .andThen(subToC3.cmd())
    //             )
    //         ).withName("ThreePieceSubMiddleEntry")
    //     );

    //     subToC3.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         c3ToM3.cmd()
    //     );

    //     c3ToM3.atTime(1.5).onTrue(intakeGamepieceNoStow());
    //     c3ToM3.done().onTrueWith(
    //         gamepiece(loop),
    //         m3ToS2.cmd(),
    //         stow()
    //     );


    //     m3ToS2.active().onTrue(aimStem(m3ToS2));
    //     m3ToS2.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         s2ToM3.cmd()
    //     );

    //     s2ToM3.active().onTrue(stow());

    //     return loop.cmd().withName("FourPieceSourceSide");
    // }

    // /**
    //  * 
    //  * 
    //  * 
    //  * @param factory The factory to create trajectories with
    //  * @return The command that represents the auto routine
    //  */
    // public Command fourPieceCloseAmpSide(ChoreoAutoFactory factory) {
    //     if (disabled) return disabledAuto();

    //     final ChoreoAutoRoutine loop = factory.newRoutine();

    //     final ChoreoAutoTrajectory ampToC1 = factory.traj(AMP.to(C1), loop);
    //     final ChoreoAutoTrajectory c1ToC2 = factory.traj(C1.to(C2), loop);
    //     final ChoreoAutoTrajectory c2ToC3 = factory.traj(C2.to(C3), loop);

    //     // entry point for the auto
    //     loop.enabled().onTrue(
    //         resetOdometry(ampToC1).andThen(
    //             Commands.parallel(
    //                 UmbrellaCommands.waitUntilSpunUp(umbrella, 5000, 0.1),
    //                 aimSub()
    //             ),
    //             feedShooter(),
    //             Commands.parallel(
    //                 intakeGamepieceNoStow(),
    //                 Commands.waitSeconds(0.2)
    //                     .andThen(ampToC1.cmd())
    //             )
    //         ).withName("ThreePieceSubMiddleEntry")
    //     );

    //     ampToC1.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         c1ToC2.cmd()
    //     );

    //     c1ToC2.active().onTrue(intakeGamepieceNoStow());
    //     c1ToC2.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         c2ToC3.cmd()
    //     );


    //     c2ToC3.active().onTrue(intakeGamepieceNoStow());
    //     c2ToC3.done().onTrueWith(
    //         gamepiece(loop),
    //         autoShoot(),
    //         stow()
    //     );

    //     return loop.cmd().withName("fourPieceCloseAmp");
    // }

    // public Command driveForward(ChoreoAutoFactory factory) {
    //     if (disabled) return disabledAuto();

    //     final ChoreoAutoRoutine loop = factory.newRoutine();

    //     final ChoreoAutoTrajectory traj = factory.traj(AMP.to(C1), loop);

    //     loop.enabled().onTrue(traj.cmd());

    //     return loop.cmd();
    // }
}
