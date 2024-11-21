package com.igknighters.util.can;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.igknighters.util.plumbing.Channel;
import com.igknighters.util.plumbing.Channel.Receiver;
import com.igknighters.util.plumbing.Channel.Sender;

import edu.wpi.first.wpilibj.Timer;

public class StatusSignalQueuer {
    public record TimestampedDouble(double timestamp, double value) {}

    private final double frequency;
    private final AtomicLong measuredFrequency = new AtomicLong();
    private final Thread thread;
    private final BaseStatusSignal[] signals;
    private final double[] previousFrequencies;
    private final Sender<TimestampedDouble>[] senders;
    private final Receiver<TimestampedDouble>[] receivers;

    @SuppressWarnings("unchecked")
    StatusSignalQueuer(int frequency, StatusSignal<?>... signals) {
        this.frequency = frequency;
        this.signals = new BaseStatusSignal[signals.length];
        this.previousFrequencies = new double[signals.length];
        final List<Sender<TimestampedDouble>> senders = new ArrayList<>();
        final List<Receiver<TimestampedDouble>> receivers = new ArrayList<>();
        for (int i = 0; i < signals.length; i++) {
            previousFrequencies[i] = signals[i].getAppliedUpdateFrequency();
            this.signals[i] = signals[i].clone();
            final Channel<TimestampedDouble> channel = new Channel<>(new TimestampedDouble[0]);
            senders.add(channel.sender());
            receivers.add(channel.openReceiver(32, Channel.ThreadSafetyMarker.CONCURRENT));
        }
        this.senders = senders.toArray(new Sender[0]);
        this.receivers = receivers.toArray(new Receiver[0]);
        this.thread = new Thread(this::run);
    }

    public Receiver<TimestampedDouble>[] start() {
        thread.start();
        for (int i = 0; i < signals.length; i++) {
            previousFrequencies[i] = signals[i].getAppliedUpdateFrequency();
        }
        BaseStatusSignal.setUpdateFrequencyForAll(frequency, signals);
        return receivers;
    }

    public void stop() {
        thread.interrupt();
        for (int i = 0; i < signals.length; i++) {
            signals[i].setUpdateFrequency(previousFrequencies[i]);
        }
    }

    public double getMeasuredFrequency() {
        return Double.longBitsToDouble(measuredFrequency.get());
    }

    private void run() {
        final Timer timer = new Timer();
        timer.start();
        double count = 0;
        final double timeout = (1.0 / frequency) * 1.5;
        while (!Thread.interrupted()) {
            BaseStatusSignal.waitForAll(timeout, signals);
            for (int i = 0; i < signals.length; i++) {
                final TimestampedDouble value = new TimestampedDouble(
                    Timer.getFPGATimestamp() - signals[i].getTimestamp().getLatency(),
                    signals[i].getValueAsDouble()
                );
                senders[i].send(value);
            }
            final double elapsed = timer.get();
            timer.reset();
            count++;
            measuredFrequency.set(Double.doubleToLongBits(count / elapsed));
        }
    }
}
