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

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import sjicontrollerinterface.JimmyInterface;

/**
 *
 * @author Haikal Pribadi <haikal.pribadi@gmail.com>
 */
public class EngineThread extends Thread {

    private JimmyInterface controller;
    private Bar bar;
    private Socket socket;
    private PrintWriter output;
    private BufferedReader input;

    EngineThread(JimmyInterface aController, Bar aBar, Socket aSocket) throws IOException {
        controller = aController;
        bar = aBar;
        socket = aSocket;
        output = new PrintWriter(socket.getOutputStream(), true);
        input = new BufferedReader(new InputStreamReader(socket.getInputStream()));
    }

    @Override
    public void run() {
        String received = null;
        int response = 0;
        try {
            while ((received = input.readLine()) != null) {
                received = received.replace("\n", "");
                received = received.replace("\r", "");
                if(!received.isEmpty()){
                    System.out.println("Server received message: " + received);
                    response = processMessage(received);
                    System.out.println("Server response: " + response);
                    if (response != -1) {
                        output.println(response);
                        output.flush();
                    }
                }
            }
        } catch (Exception e) {
            System.out.println(e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }

    public int processMessage(String received) {
        int response;

        String[] command = received.split(" ");
        if(command.length < 1)
            return -1;
        
        if (command[0].equalsIgnoreCase("accept") && command.length == 3) {
            response = acceptDrink(command[1], command[2]);

        } else if (command[0].equalsIgnoreCase("change") && command.length == 4) {
            response = changeDrink(command[1], command[2], command[3]);

        } else if (command[0].equalsIgnoreCase("changeconfirm") && command.length == 4){
            bar.changeDrink(command[1], command[2]);
            bar.orderDrink(command[1], command[3]);
            response = -1;
            
        } else if (command[0].equalsIgnoreCase("get") && command.length == 3) {
            response = getDrink(command[1], command[2]);

        } else if (command[0].equalsIgnoreCase("getconfirm") && command.length == 3){
            bar.orderDrink(command[1], command[3]);
            response = -1;
            
        } else if (command[0].equalsIgnoreCase("incorrect") && command.length == 3) {
            response = incorrectDrink(command[1], command[2]);

        } else if (command[0].equalsIgnoreCase("incorrectconfirm") && command.length == 3){
            bar.incorrectDrink(command[1], command[2]);
            response = -1;
            
        } else if (command[0].equalsIgnoreCase("reject") && command.length == 3) {
            response = rejectDrink(command[1], command[2]);

        } else if (command[0].equalsIgnoreCase("rejectconfirm") && command.length == 3){
            bar.rejectDrink(command[1], command[2]);
            response = -1;
            
        } else if (command[0].equalsIgnoreCase("produced") && command.length == 3) {
            response = controller.producedDrinkIsCorrect(command[1], command[2]) ? 1 : 0;

        } else if (command[0].equalsIgnoreCase("getTotalAccepted") && command.length==2){
            response = bar.getTotalAccepted(command[1]);
            
        } else if (command[0].equalsIgnoreCase("getTotalAccepted") && command.length==3){
            response = bar.getTotalAccepted(command[1], command[2]);
            
        } else if (command[0].equalsIgnoreCase("getTotalChangeRequests") && command.length==2){
            response = bar.getTotalChangeRequests(command[1]);
            
        } else if (command[0].equalsIgnoreCase("getTotalChangeRequests") && command.length==3){
            response = bar.getTotalChangeRequests(command[1], command[2]);
            
        } else if (command[0].equalsIgnoreCase("getTotalConsumed") && command.length==2){
            response = bar.getTotalDrinksConsumed(command[1]);
            
        } else if (command[0].equalsIgnoreCase("getTotalConsumed") && command.length==3){
            response = bar.getTotalDrinksConsumed(command[1], command[2]);
            
        } else if (command[0].equalsIgnoreCase("getTotalOrdered") && command.length==2){
            response = bar.getTotalDrinksOrdered(command[1]);
            
        } else if (command[0].equalsIgnoreCase("getTotalOrdered") && command.length==3){
            response = bar.getTotalDrinksOrdered(command[1], command[2]);
            
        } else if (command[0].equalsIgnoreCase("getTotalUnconsumed") && command.length==2){
            response = bar.getTotalDrinksUnconsumed(command[1]);
            
        } else if (command[0].equalsIgnoreCase("getTotalUnconsumed") && command.length==3){
            response = bar.getTotalDrinksUnconsumed(command[1], command[2]);
            
        } else if (command[0].equalsIgnoreCase("getTotalIncorrectClaims") && command.length==2){
            response = bar.getTotalIncorrectClaims(command[1]);
            
        } else if (command[0].equalsIgnoreCase("getTotalIncorrectClaims") && command.length==3){
            response = bar.getTotalIncorrectClaims(command[1], command[2]);
            
        } else if (command[0].equalsIgnoreCase("getTotalRejected") && command.length==2){
            response = bar.getTotalRejected(command[1]);
            
        } else if (command[0].equalsIgnoreCase("getTotalRejected") && command.length==3){
            response = bar.getTotalRejected(command[1], command[2]);
            
        }
        else {
            response = invalidMessage(received);

        }
        return response;
    }

    private int acceptDrink(String username, String drinkType) {
        int response = -1;
        controller.acceptDrink(username, drinkType);
        bar.accept(username, drinkType);
        return response;
    }

    private int changeDrink(String username, String drinkType1, String drinkType2) {
        int response = controller.changeDrink(username, drinkType2);
        if (response == JimmyInterface.COMING_RIGHT_UP
                || response == JimmyInterface.YES_SIR
                || response == JimmyInterface.SAY_NOTHING) {
            bar.changeDrink(username, drinkType1);
            bar.orderDrink(username, drinkType2);
        }
        return response;
    }

    private int getDrink(String username, String drinkType) {
        int response = controller.getDrink(username, drinkType);
        if (response == JimmyInterface.COMING_RIGHT_UP
                || response == JimmyInterface.YES_SIR
                || response == JimmyInterface.SAY_NOTHING) {
            bar.orderDrink(username, drinkType);
        }
        return response;
    }

    private int incorrectDrink(String username, String drinkType) {
        int response = controller.incorrectDrink(username, drinkType);
        if (response == JimmyInterface.COMING_RIGHT_UP
                || response == JimmyInterface.APOLOGY
                || response == JimmyInterface.YES_SIR
                || response == JimmyInterface.SAY_NOTHING) {
            bar.incorrectDrink(username, drinkType);
        }
        return response;
    }

    private int rejectDrink(String username, String drinkType) {
        int response = controller.rejectDrink(username, drinkType);
        if (response == JimmyInterface.COMING_RIGHT_UP
                || response == JimmyInterface.APOLOGY
                || response == JimmyInterface.YES_SIR
                || response == JimmyInterface.SAY_NOTHING) {
            bar.rejectDrink(username, drinkType);
        }
        return response;
    }
    
    private int invalidMessage(String message){
        System.err.println("Jimmy Engine received an invalid message: " + message);
        return -1;
    }
}
