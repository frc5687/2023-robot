package org.frc5687.lib.vision;

import java.nio.ByteBuffer;
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

    public VisionProcessor() {
        context = ZMQ.context(1);
        subscribers = new HashMap<>();
        publishers = new HashMap<>();
        executor = Executors.newSingleThreadExecutor();
    }

    public synchronized void createSubscriber(String topic) {
        ZMQ.Socket subscriber = context.socket(SocketType.SUB);
        subscriber.bind("tcp://*:5555");
        subscriber.subscribe(topic.getBytes());
        subscribers.put(topic, subscriber);
    }

    public synchronized void createPublisher(String topic) {
        ZMQ.Socket publisher = context.socket(SocketType.PUB);
        publisher.bind("tcp://*:5556");
        publishers.put(topic, publisher);
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
//                processData(topic, data);
            }
        }
    }

    private void processData(String topic) {
        // Your code to process the data goes here
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
        for (int i = 0; i < numElements; i++) {
            floatArray[i] = buffer.getFloat();
        }
        return floatArray;
    }
}