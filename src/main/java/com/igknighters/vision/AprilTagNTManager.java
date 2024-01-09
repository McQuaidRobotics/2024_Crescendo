package com.igknighters.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class AprilTagNTManager {
    private class NTTag {
        public double x, y, z, yaw, pitch, roll;
        public Integer id;
        private NetworkTable table;

        public NTTag(String name, Integer id, Pose3d pose) {
            x = pose.getX();
            y = pose.getY();
            z = pose.getZ();
            yaw = Units.radiansToDegrees(pose.getRotation().getZ());
            this.id = id;
            table = NetworkTableInstance.getDefault().getTable("/AprilTags").getSubTable(name);

            table.putValue("pose/x", NetworkTableValue.makeDouble(x));
            table.putValue("pose/y", NetworkTableValue.makeDouble(y));
            table.putValue("pose/z", NetworkTableValue.makeDouble(z));
            table.putValue("rot/yaw", NetworkTableValue.makeDouble(yaw));
            table.putValue("rot/pitch", NetworkTableValue.makeDouble(pitch));
            table.putValue("rot/roll", NetworkTableValue.makeDouble(roll));
            table.putValue("id", NetworkTableValue.makeInteger(id));
        }

        public NTTag(String name, AprilTag tag) {
            this(name, tag.ID, tag.pose);
        }

        public void update() {
            x = table.getEntry("pose/x").getDouble(x);
            y = table.getEntry("pose/y").getDouble(y);
            z = table.getEntry("pose/z").getDouble(z);
            yaw = table.getEntry("rot/yaw").getDouble(yaw);
            yaw = table.getEntry("rot/pitch").getDouble(pitch);
            yaw = table.getEntry("rot/roll").getDouble(roll);
            id = (int) table.getEntry("id").getInteger(id);
        }

        public AprilTag toAprilTag() {
            return new AprilTag(id, new Pose3d(
                new Translation3d(x, y, z),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(yaw))
            ));
        }

        public void cleanup() {
            table.getEntry("pose/x").unpublish();
            table.getEntry("pose/y").unpublish();
            table.getEntry("pose/z").unpublish();
            table.getEntry("rot/yaw").unpublish();
            table.getEntry("rot/pitch").unpublish();
            table.getEntry("rot/roll").unpublish();
            table.getEntry("id").unpublish();
        }
    }

    private static String numToLetter(int num) {
        return ((char) (num + 65)) + "";
    }

    private List<NTTag> tags = new ArrayList<>();
    private final NetworkTable tagTable = NetworkTableInstance.getDefault().getTable("/AprilTags");
    private final NetworkTableEntry addTagBool, removeTagBool;

    public AprilTagNTManager(List<AprilTag> startingFieldLayout) {
        for (AprilTag tag : startingFieldLayout) {
            var name = "Tag_"+numToLetter(tags.size());
            tags.add(new NTTag(name, tag));
        }

        addTagBool = tagTable.getEntry("addTag");
        addTagBool.setDefaultBoolean(false);
        removeTagBool = tagTable.getEntry("removeTag");
        removeTagBool.setDefaultBoolean(false);
    }

    public void update() {
        if (addTagBool.getBoolean(false)) {
            var name = "Tag_"+numToLetter(tags.size());
            tags.add(new NTTag(name, tags.size()+1, new Pose3d()));
            addTagBool.setBoolean(false);
        }
        if (removeTagBool.getBoolean(false)) {
            tags.remove(tags.size()-1).cleanup();
            removeTagBool.setBoolean(false);
        }
        for (NTTag tag : tags) {
            tag.update();
        }
    }

    public AprilTagFieldLayout generateField() {
        ArrayList<AprilTag> tagClone = new ArrayList<>();
        tags.stream().map(NTTag::toAprilTag).forEach(tagClone::add);
        return new AprilTagFieldLayout(tagClone, 100.0, 100.0);
    }
}
