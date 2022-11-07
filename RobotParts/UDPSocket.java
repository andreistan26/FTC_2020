package RobotParts;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

import static RobotParts.RobotPosition.robotPose;

public class UDPSocket extends Thread {
    private DatagramSocket socket;
    private byte[] buf;
    private final static int SOCKET_PORT = 6969;
    private final static int PACKET_PORT = 6970;
    private boolean running = true;

    public UDPSocket() throws SocketException{
        socket = new DatagramSocket(SOCKET_PORT);
        buf = new byte[256];
    }

    public void run(){
        InetAddress address = null;
        try{
            address = InetAddress.getLocalHost();
        }catch (UnknownHostException e){
            e.printStackTrace();
        }
        while(running){
            String msg = robotPose.x + "," + robotPose.y + "," + robotPose.angle;
            buf = msg.getBytes();
            DatagramPacket packet = new DatagramPacket(buf, buf.length,address, PACKET_PORT);
            try {
                socket.send(packet);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        socket.close();
    }

    public void terminate(){
        running = false;
    }
}
