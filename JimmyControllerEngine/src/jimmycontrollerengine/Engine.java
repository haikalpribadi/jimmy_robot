/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Haikal Pribadi <haikal.pribadi@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the Haikal Pribadi nor the names of other
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package jimmycontrollerengine;

import java.io.File;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URL;
import java.net.URLClassLoader;
import sjicontrollerinterface.Customer;
import sjicontrollerinterface.JimmyInterface;

/**
 *
 * @author Haikal Pribadi <haikal.pribadi@gmail.com>
 */
public class Engine {
    
    public static String instruction = "Please provide appropriate arguments. Usage example:"
            + "\njava -jar jimmycontrollerengine.jar controller_jar [port_number]"
            + "\n"
            + "\ncontroller_jar: JAR containing Controller.java that implements JimmyInterface"
            + "\nport_number: (optional) port number for JimmyEngine to communicate with Jimmy Robot. "
            + "If not provided, the default port number (41000) will be used."
            + "\n";
    
    private int port;
    private JimmyInterface controller;
    private ServerSocket serverSocket;
    private Bar bar;
    
    Engine(){
        controller = null;
        port = 41000;
        bar = new Bar();
        Customer.setPort(port);
    }
    
    Engine(int portNum){
        controller = null;
        port = portNum;
        bar = new Bar();
        Customer.setPort(port);
    }
    
    public void setPort(int portNum){
        port = portNum;
        Customer.setPort(port);
    }
    
    public int getPort(){
        return port;
    }
    
    public void loadController(File controllerJAR) {
        URLClassLoader classLoader = null;
        JimmyInterface jimmyController = null;

        try {
            String className = "sjicontroller.Controller";
            URL jarURL = new URL("file:" + controllerJAR.getAbsolutePath());
            classLoader = new URLClassLoader(new URL[]{jarURL});
            Class loadedClass = classLoader.loadClass(className);
            jimmyController = (JimmyInterface) loadedClass.newInstance();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println();
            System.out.println(instruction);
        }

        controller = jimmyController;
    }
    
    public void execute(){
        if(controller==null){
            System.err.print("Unable to start Engine execution,");
            System.err.println("because Jimmy Controller has not been loaded");
            return;
        }
        try{
            serverSocket = new ServerSocket(port);
            while(true){
                Socket socket = serverSocket.accept();
                Thread thread = new Thread(new EngineThread(controller, bar, socket));
                thread.start();
            }
        } catch(Exception e){
            System.out.println(e.getMessage());
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {

        if (args.length == 0 || "-h".equals(args[0]) || "-help".equals(args[0])) {
            System.out.println(instruction);
            System.exit(0);
        }

        Engine engine = new Engine();
        if (args.length >= 1) {
            try {
                File jar = new File(args[0]);
                engine.loadController(jar);
                System.out.println("Using controller from: " + jar.getName());
            } catch (Exception e) {
                System.out.println(e.getMessage());
                System.out.println(instruction);
            }
        }
        if (args.length == 2) {
            try {
                int port = Integer.parseInt(args[1]);
                engine.setPort(port);
                System.out.println("Using port number: " + port);
            } catch (Exception e) {
                System.out.println(e.getMessage());
                e.printStackTrace();
                System.out.println();
                System.out.println(instruction);
                System.exit(0);
            }
        }
        
        engine.execute();
    }
}
