package org.frc5687.lib.vision;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.chargedup.util.SubsystemManager;
import org.zeromq.SocketType;
import org.zeromq.ZMQ;

public class VisionProcessor {
    private final ZMQ.Context context;
    private final Map<String, ZMQ.Socket> subscribers;
    private final Map<String, ZMQ.Socket> publishers;
    private volatile boolean running = true;

    private final Notifier _receiveNotifier =
            new Notifier(
                    () -> {
                        synchronized (VisionProcessor.this) {
                            receive();
                        }
                    });

    private ArrayList<TrackedObjectInfo> _trackedTargets;

    public VisionProcessor() {
        context = ZMQ.context(1);
        subscribers = new HashMap<>();
        publishers = new HashMap<>();
        _trackedTargets = new ArrayList<>();
    }

    public synchronized void createSubscriber(String topic, String addr) {
        ZMQ.Socket subscriber = context.socket(SocketType.SUB);
        subscriber.connect(addr);
        subscriber.subscribe(topic.getBytes());
        subscribers.put(topic, subscriber);
    }

    public synchronized void createPublisher(String topic) {
//        ZMQ.Socket publisher = context.socket(SocketType.PUB);
//        publisher.bind("tcp://*:5556");
//        publishers.put(topic, publisher);
    }

    public void start() {
        running = true;
        _receiveNotifier.startPeriodic(0.02); // Vision system only runs at 20 ms atm
    }

    public void stop() {
        running = false;
        _receiveNotifier.stop();
    }

    private void receive() {
        for (Map.Entry<String, ZMQ.Socket> entry : subscribers.entrySet()) {
            ZMQ.Socket subscriber = entry.getValue();
            String topic = subscriber.recvStr();
            byte[] serializedData = subscriber.recv();
            processData(topic, serializedData);
        }
    }

    private synchronized void processData(String topic, byte[] data) {
        if (topic.equals("vision")) {
            _trackedTargets.clear();
            decodeToTrackedObjectInfoList(data, _trackedTargets);
        }
    }

    public synchronized ArrayList<TrackedObjectInfo> getTrackedObjects() {
        return _trackedTargets;
    }

    public synchronized void sendData(String topic) {
        ZMQ.Socket publisher = publishers.get(topic);
        if (publisher != null) {
//            publisher.sendMore(topic.getBytes());
//            publisher.send(serializedData);
        } else {
            System.out.println("Publisher for topic " + topic + " does not exist.");
        }
    }
    /* encode the float array to a byte array for ZMQ */
    private static byte[] encodeFloatArray(float[] floatArray) {
        ByteBuffer byteBuffer = ByteBuffer.allocate(floatArray.length * Float.BYTES);
        for (float f : floatArray) {
            byteBuffer.putFloat(f);
        }
        return byteBuffer.array();
    }
    /* Decode the incoming packet from ZMQ */
    private static void decodeToTrackedObjectInfoList(byte[] byteArray, ArrayList<TrackedObjectInfo> list) {
        int numElements = byteArray.length / TrackedObjectInfo.sizeBytes();
        ByteBuffer buffer = ByteBuffer.wrap(byteArray);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        for (int i = 0; i < numElements; i++) {
            int id = buffer.get();
            list.add(new TrackedObjectInfo(
                    TrackedObjectInfo.GameElement.valueOf(id),
                    buffer.getFloat(),
                    buffer.getFloat(),
                    buffer.getFloat(),
                    buffer.getFloat(),
                    buffer.getFloat(),
                    buffer.getFloat()
            ));
        }
    }
}