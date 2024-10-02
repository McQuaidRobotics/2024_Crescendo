package com.igknighters.util.logging;

import com.igknighters.constants.ConstValues.kChannels;
import com.igknighters.util.plumbing.Channels.Receiver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import monologue.Monologue;

public class GlobalField {
    public static final String NOTES_POSITION = "FieldNotes";
    public static final String MODULES_POSITION = "FieldModules";
    public static final String SEEN_TAGS_POSITION = "FieldTags";
    public static final String ARBITRARY_TARGETS = "ArbitraryTargets";

    private static final Field2d field = new Field2d();
    private static final FieldObject2d robot = field.getRobotObject();
    private static final FieldObject2d modules = field.getObject(MODULES_POSITION);
    private static final FieldObject2d notes = field.getObject(NOTES_POSITION);
    private static final FieldObject2d seenTags = field.getObject(SEEN_TAGS_POSITION);
    private static final FieldObject2d arbitraryTargets = field.getObject(ARBITRARY_TARGETS);


    public static void enable(){
        Receiver.reactor(
            kChannels.POSITION,
            Pose2d.class,
            poses -> robot.setPose(poses)
        );
        Receiver.reactor(
            MODULES_POSITION,
            Pose2d[].class,
            poses -> modules.setPoses(poses)
        );
        Receiver.reactor(
            NOTES_POSITION,
            Pose2d[].class,
            poses -> notes.setPoses(poses)
        );
        Receiver.reactor(
            SEEN_TAGS_POSITION,
            Pose2d[].class,
            poses -> seenTags.setPoses(poses)
        );
        Receiver.reactor(
            ARBITRARY_TARGETS,
            Pose2d[].class,
            poses -> arbitraryTargets.setPoses(poses)
        );

        Monologue.publishSendable("/Visualizers/Field", field);
    }
}
