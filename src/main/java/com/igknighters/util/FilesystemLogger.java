package com.igknighters.util;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.BufferOverflowException;
import java.nio.CharBuffer;
import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;

import edu.wpi.first.wpilibj.DriverStation;
import monologue.Monologue;

public class FilesystemLogger {
    private static record FileReader(
        String fileName,
        BufferedReader reader,
        CharBuffer buffer
    ) {}

    private static record FileData(
        String fileName,
        String lines
    ) {}

    private final ArrayBlockingQueue<String> filePaths = new ArrayBlockingQueue<>(8);
    private final ArrayBlockingQueue<FileData> queue = new ArrayBlockingQueue<>(64);

    private final Thread thread;

    public FilesystemLogger(String... paths) {
        thread = new Thread(this::run);
        thread.setDaemon(true);
        thread.start();

        for (String path : paths) {
            addFile(path);
        }
    }

    private void run() {
        final ArrayList<FileReader> readers = new ArrayList<>();
        while (true) {
            for (final String path : filePaths) {
                try {
                    String fileName = path.substring(path.lastIndexOf('/') + 1);
                    readers.add(new FileReader(
                        fileName,
                        new BufferedReader(new java.io.FileReader(path)),
                        CharBuffer.allocate(10240)
                    ));
                } catch (FileNotFoundException e) {
                    DriverStation.reportError("[FilesystemLogger] Failed to open file \"" + path + "\", skipping capture.", true);
                    return;
                }
            }
            filePaths.clear();

            for (final var reader : readers) {
                while (true) {
                    int nextChar = -1;
                    try {
                        nextChar = reader.reader.read();
                    } catch (IOException  e) {
                        DriverStation.reportError(
                            "[FilesystemLogger] Failed to read from file \""
                                + reader.fileName
                                + "\", skipping capture.",
                            true
                        );
                        return;
                    }
                    if (nextChar == -1) {
                        break;
                    }
                    try {
                        reader.buffer.put((char) nextChar);
                    } catch (BufferOverflowException e) {}
                }

                var buffer = reader.buffer;
                String output = null;
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
                if (output != null) {
                    try {
                        queue.put(new FileData(reader.fileName, output));
                    } catch (InterruptedException e) {
                        try {
                            reader.reader.close();
                        } catch (IOException io) {}
                        return;
                    }
                }
            }

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                try {
                    for (var reader : readers) {
                        reader.reader.close();
                    }
                } catch (IOException io) {}
                return;
            }

        }
    }

    public void addFile(String path) {
        filePaths.add(path);
    }

    public void log() {
        while (!queue.isEmpty()) {
            FileData data = queue.poll();
            if (data == null) {
                return;
            }
            Monologue.log("/Filesystem/" + data.fileName, data.lines);
        }
    }
}
