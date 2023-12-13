package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Robot;

public class AutoProfiler {
    public static class AutoEntry {
        public final String name;
        public final Double time;
        public final Boolean interupted;

        public AutoEntry(String name, Double time, Boolean interupted) {
            this.name = name;
            this.time = time;
            this.interupted = interupted;
        }
    }

    private static List<AutoEntry> entries = new ArrayList<>();

    public static void addEntry(String name, Double time, Boolean interupted) {
        entries.add(new AutoEntry(name, time, interupted));
    }

    public static void clearEntries() {
        entries.clear();
    }

    static final String PATH = "/home/lvuser/autos";
    public static void writeToFile() {
        if (Robot.isReal()) {
            //find all files that start with PATH and then get number after it
            //get max number and add 1 to it
            //write to file with that number
            int number = 0;
            for (var file : new java.io.File(PATH).listFiles()) {
                if (file.getName().startsWith(PATH)) {
                    try {
                        int num = Integer.parseInt(
                            file.getName()
                                .substring(PATH.length(), file.getName().length() - 4)
                        );
                        if (num > number) {
                            number = num;
                        }
                    } catch (Exception e) {
                        System.out.println("Error parsing file name: " + e);
                    }
                }
            }
            try {
                java.io.FileWriter writer = new java.io.FileWriter(PATH + (number + 1) + ".csv");
                writer.write("name,time,interupted\n");
                for (AutoEntry entry : entries) {
                    writer.write(entry.name + "," + entry.time + "," + entry.interupted + "\n");
                }
                writer.close();
            } catch (Exception e) {
                System.out.println("Error writing to file: " + e);
            }
        }
    }
}
