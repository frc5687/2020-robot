package org.frc5687.infiniterecharge.robot.util;

import org.frc5687.infiniterecharge.robot.Constants;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;

public class RobotProxy extends OutliersProxy {

    private ProxyThread _thread;
    private int _period = Constants.Proxy.PERIOD;
    private int _jetsonPort = Constants.Proxy.JETSONPORT;
    private final static String _jetsonServerHost = "172.17.0.27";
    private int _roboRioPort = Constants.Proxy.ROBORIOPORT;


    public RobotProxy(){
        try {
            error("staring proxy");
            _thread = new ProxyThread();
            _thread.start();
        } catch (IOException ioe){

        }
    }

    @Override
    public void updateDashboard() {
    }

    public void stop(){
        _thread.close();
    }

    public void start() {
        _thread.open();
    }

    public void sendData(Frame frame){
        try {
            _thread.sendData(frame);
        } catch(IOException ioe){
            error(ioe.getMessage());
        }
    }


    protected class ProxyThread extends Thread{
        protected DatagramSocket rioSocket = null;
        protected DatagramSocket jetsonSocket = null;
        protected InetAddress jetsonHost = null;
        private volatile boolean StopServer = false;

        private void onData(String data) {
            error("calling onData");
            String toPrint = "java got data: " + data;
           metric("Data", toPrint);
        }

        public ProxyThread() throws IOException {
            rioSocket = new DatagramSocket(_roboRioPort);
            rioSocket.setSoTimeout(250);
            jetsonHost = InetAddress.getByName(_jetsonServerHost);
            jetsonSocket = new DatagramSocket();
        }

        public void sendData(Frame frame) throws IOException {
            long rioMillis  = System.currentTimeMillis();
            StringBuilder buffer = new StringBuilder();
            buffer.append(Long.toString(rioMillis));
            buffer.append(";");
            buffer.append(Double.toString(frame.getVelocity()));
            buffer.append(";");
            buffer.append(Double.toString(frame.getOmega()));
            buffer.append(";");
            buffer.append(Float.toString(frame.getYaw()));
            buffer.append(";");
            byte[] data = buffer.toString().getBytes();
            DatagramPacket packet = new DatagramPacket(data, data.length, jetsonHost, _jetsonPort);
            jetsonSocket.send(packet);
        }

        public void run() {
            try {
                while(!StopServer){
                    byte buf[] = new byte[Constants.Proxy.PACKET_SIZE];
                    DatagramPacket packet = new DatagramPacket(buf, buf.length);
//                    error("packet is at " + packet.etAddress());
                    try {
                        rioSocket.receive(packet);
                        if(packet == null) {
                            error("packet is null");
                        }
                        String data = new String(packet.getData(), 0, packet.getLength());
                        onData(data);
                    } catch(SocketTimeoutException e){
                    }
                }
                rioSocket.close();
            } catch (IOException ioe) {
                ioe.printStackTrace();
            }
        }

        public void close() {
            StopServer = true;
        }
        public void open() {
            StopServer = false;
        }
    }
}

