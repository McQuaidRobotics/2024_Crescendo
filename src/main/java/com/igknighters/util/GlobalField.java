package com.igknighters.util;

import com.igknighters.constants.ConstValues.kChannels;
import com.igknighters.util.Channels.Receiver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class GlobalField {
    public static final String NOTES_POSITION = "FieldNotes";
    public static final String MODULES_POSITION = "FieldModules";
    public static final String SEEN_TAGS_POSITION = "FieldTags";

    private static final Field2d field = new Field2d();
    private static final FieldObject2d robot = field.getRobotObject();
    private static final FieldObject2d modules = field.getObject(MODULES_POSITION);
    private static final FieldObject2d notes = field.getObject(NOTES_POSITION);
    private static final FieldObject2d seenTags = field.getObject(SEEN_TAGS_POSITION);
    static {
        Receiver.reactor(
            kChannels.POSITION,
            Pose2d.class,
            pose -> robot.setPose(pose)
        );
        Receiver.reactor(
            MODULES_POSITION,
            Pose2d[].class,
            pose -> modules.setPoses(pose)
        );
        Receiver.reactor(
            NOTES_POSITION,
            Pose2d[].class,
            pose -> notes.setPoses(pose)
        );
        Receiver.reactor(
            SEEN_TAGS_POSITION,
            Pose2d[].class,
            pose -> seenTags.setPoses(pose)
        );
    }
}
