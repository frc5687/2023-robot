package org.frc5687.lib.vision;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.zeromq.SocketType;
import org.zeromq.ZMQ;

public class VisionProcessor {
    private final ZMQ.Context context;
    private final ExecutorService executor;
    private final Map<String, ZMQ.Socket> subscribers;
    private final Map<String, ZMQ.Socket> publishers;
    private volatile boolean running = true;

    private ArrayList<TrackedObjectInfo> _trackedTargets;

    public VisionProcessor() {
        context = ZMQ.context(1);
        subscribers = new HashMap<>();
        publishers = new HashMap<>();
        executor = Executors.newSingleThreadExecutor();
        _trackedTargets = new ArrayList<TrackedObjectInfo>();
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
        executor.execute(this::receive);
    }

    public void stop() {
        running = false;
        executor.shutdown();
    }

    private void receive() {
        while (running) {
            for (Map.Entry<String, ZMQ.Socket> entry : subscribers.entrySet()) {
                ZMQ.Socket subscriber = entry.getValue();
                String topic = entry.getKey();
                byte[] serializedData = subscriber.recv();
                processData(topic, serializedData);
            }
        }
    }

    private synchronized void processData(String topic, byte[] data) {
        if (topic.equals("vision")) {
            float[] vision_data = decodeToFloatArray(data);
            _trackedTargets.add(new TrackedObjectInfo(
                    TrackedObjectInfo.GameElement.valueOf((int)vision_data[0]),
                    vision_data[1],
                    vision_data[2],
                    vision_data[3],
                    vision_data[4],
                    vision_data[5],
                    vision_data[6]
            ));
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
    private static float[] decodeToFloatArray(byte[] byteArray) {
        int numElements = byteArray.length / Float.BYTES;
        float[] floatArray = new float[numElements];
        ByteBuffer buffer = ByteBuffer.wrap(byteArray);
        floatArray[0] = buffer.getInt();
        for (int i = 1; i < numElements; i++) {
            floatArray[i] = buffer.getFloat();
        }
        return floatArray;
    }
}