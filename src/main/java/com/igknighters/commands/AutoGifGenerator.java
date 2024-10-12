package com.igknighters.commands;

import java.io.File;
import java.util.ArrayList;
import java.util.Optional;

import com.fasterxml.jackson.core.util.DefaultPrettyPrinter;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectWriter;
import com.igknighters.Localizer;
import com.igknighters.constants.ConstValues.kChannels;
import com.igknighters.util.plumbing.GlobalChannels.Receiver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoGifGenerator {
    @SuppressWarnings("unused")
    public static Command convertToJson(String autoName, Localizer localizer) {
        // Receiver<Pose2d> robotPoseReceiver = Receiver.latest(kChannels.POSITION_SWERVE, Pose2d.class);
        Receiver<Translation2d> noteReciever = Receiver.latest(kChannels.PICKED_UP_NOTES, Translation2d.class);
        ArrayList<Pose2d> poseList = new ArrayList<>();
        ArrayList<Object> noteList = new ArrayList<>();
        var command = Commands.run(() -> {
            poseList.add(localizer.pose());
        
            Optional<Translation2d> noteOptional = noteReciever.tryRecv();
            if (noteOptional.isPresent()) {
                Translation2d note = noteOptional.get();
                var data = new Object() {
                    public int pose_index = poseList.size();
                    public Translation2d note_translation = note;
                };
                noteList.add(data);
            }
        }).finallyDo(() -> {
            ObjectMapper objectMapper = new ObjectMapper();
            ObjectWriter objectWriter = objectMapper.writer(new DefaultPrettyPrinter());
            var data = new Object() {
                public ArrayList<Pose2d> poses = poseList;
                public ArrayList<Object> notes = noteList;
            };
            try {
                objectWriter.writeValue(new File("./autogifgenerator/autos/" + autoName + ".meow"), data);
            } catch (Exception e) {
                DriverStation.reportError(e.getMessage(), e.getStackTrace());
            }
        });

        return command;
    }
}
