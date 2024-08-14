package com.igknighters.util.logging;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.BufferOverflowException;
import java.nio.CharBuffer;
import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.ArrayBlockingQueue;

import com.igknighters.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import monologue.Monologue;

/**
 * An interface to automatically log the contents of a file to datalog and network tables
 */
public class FilesystemLogger {
    private static record FileReader(
        String name,
        BufferedReader reader,
        CharBuffer buffer,
        double period,
        Timer timer
    ) {
        /**
         * @return The time left until the next period (in milliseconds)
         */
        public long periodLeft() {
            return (long) ((period - timer.get()) * 1000);
        }
    }

    private static record FileData(
        String fileName,
        String lines
    ) {}

    private static record FileDescriptor(
        String path,
        String name,
        double period
    ) {}

    private final ArrayBlockingQueue<FileDescriptor> newFiles = new ArrayBlockingQueue<>(8);
    private final ArrayBlockingQueue<FileData> queue = new ArrayBlockingQueue<>(64);

    private final Thread thread;

    public FilesystemLogger() {
        thread = new FilesystemLoggerThread();
    }

    private class FilesystemLoggerThread extends Thread {
        final ArrayList<FileReader> files = new ArrayList<>();

        public FilesystemLoggerThread() {
            super("FilesystemLogger");
            setDaemon(true);
        }

        private Optional<FileReader> openFile(FileDescriptor desc){
            try {
                var timer = new Timer();
                timer.start();
                return Optional.of(new FileReader(
                    desc.name,
                    Robot.isReal()
                        ? new BufferedReader(new java.io.FileReader(desc.path))
                        : null,
                    CharBuffer.allocate(32768),
                    desc.period,
                    timer
                ));
            } catch (FileNotFoundException e) {
                DriverStation.reportError("[FilesystemLogger] Failed to open file \"" + desc + "\", skipping capture.", true);
                return Optional.empty();
            }
        };

        private void closeReaders() {
            if (Robot.isSimulation()) return;
            for (var reader : files) {
                try {
                    reader.reader.close();
                } catch (IOException e) {}
            }
        };

        private void openNewFiles() {
            while (!newFiles.isEmpty()) {
                try {
                    openFile(newFiles.take()).ifPresent(files::add);
                } catch (InterruptedException e) {
                    closeReaders();
                    return;
                }
            }
        };

        private FileReader nextFile() {
            long minTime = Long.MAX_VALUE;
            FileReader next = null;
            for (var reader : files) {
                long timeLeft = reader.periodLeft();
                if (timeLeft < minTime) {
                    minTime = timeLeft;
                    next = reader;
                }
            }
            return next;
        };

        private boolean waitForPeriodToExpire(FileReader reader){
            try {
                Thread.sleep(Math.max(0, reader.periodLeft()));
            } catch (InterruptedException e) {
                return false;
            }
            return true;
        };

        private String readFile(FileReader file) {
            while (true) {
                int nextChar = -1;
                try {
                    nextChar = file.reader.read();
                } catch (IOException  e) {
                    DriverStation.reportError(
                        "[FilesystemLogger] Failed to read from file \""
                            + file.name
                            + "\", skipping capture.",
                        true
                    );
                    return "";
                }
                if (nextChar == -1) {
                    break;
                }
                try {
                    file.buffer.put((char) nextChar);
                } catch (BufferOverflowException e) {}
            }

            var buffer = file.buffer;
            String output = "";
            for (int i = buffer.position(); i > 0; i--) {
                if (i < buffer.position() && buffer.get(i) == '\n') {
                    int originalPosition = buffer.position();
                    output = new String(buffer.array(), 0, i);
                    buffer.rewind();
                    buffer.put(buffer.array(), i + 1, buffer.limit() - i - 1);
                    buffer.position(originalPosition - i - 1);
                    break;
                }
            }

            return output;
        }

        private String readFileSimple(FileReader file) {
            return "Simulated file data " + Double.toHexString(Math.random());
        }

        @Override
        public void run() {
            while (true) {
                openNewFiles();

                var file = nextFile();

                waitForPeriodToExpire(file);

                String output = Robot.isReal()
                    ? readFile(file)
                    : readFileSimple(file);
                if (!output.isEmpty()) {
                    try {
                        queue.put(new FileData(file.name, output));
                    } catch (InterruptedException e) {
                        closeReaders();
                        return;
                    }
                }

                openNewFiles();

                file.timer.restart();
            }
        }
    }

    /**
     * Add a file to be logged
     * 
     * @param path The path to the file
     * @param name The name to log the file as
     * @param period The period to log the file at (in seconds)
     */
    public void addFile(String path, String name, double period) {
        newFiles.add(new FileDescriptor(path, name, period));
        if (!thread.isAlive()) {
            thread.start();
        }
    }

    /**
     * Add a file to be logged
     * 
     * @param path The path to the file
     * @param period The period to log the file at (in seconds)
     */
    public void addFile(String path, double period) {
        String name = path.substring(path.lastIndexOf('/') + 1);
        newFiles.add(new FileDescriptor(path, name, period));
        if (!thread.isAlive()) {
            thread.start();
        }
    }

    /**
     * Polls the reader thread and pushes the data to the network tables/datalog
     */
    public void log() {
        while (!queue.isEmpty()) {
            FileData data = queue.poll();
            if (data == null) {
                return;
            }
            // If you don't use monologue just use normal network table entries in a hashmap or whatever
            Monologue.log("/Filesystem/" + data.fileName, data.lines);
        }
    }
}
