package sjicontrollerinterface;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.Scanner;

/*
 * @author Marc Davies
 */
public abstract class Customer {

    private static String SERVER_IP = "127.0.0.1";
    private static int SERVER_PORT = 41000;

    public static void setPort(int port) {
        SERVER_PORT = port;
    }

    public static int getTotalAccepted(String username) {
        String user = username.replace(" ", "_");

        int value = processMessage(username, "getTotalAccepted " + user);

        return value;
    }

    public static int getTotalAccepted(String username, String drinkType) {
        String user = username.replace(" ", "_");
        String drink = drinkType.replace(" ", "_");

        int value = processMessage(username, "getTotalAccepted " + user + " " + drink);

        return value;
    }

    public static int getTotalChangeRequests(String username) {
        String user = username.replace(" ", "_");

        int value = processMessage(username, "getTotalChangeRequests " + user);

        return value;
    }

    public static int getTotalChangeRequests(String username, String drinkType) {
        String user = username.replace(" ", "_");
        String drink = drinkType.replace(" ", "_");

        int value = processMessage(username, "getTotalChangeRequests " + user + " " + drink);

        return value;
    }

    public static int getTotalDrinksConsumed(String username) {
        String user = username.replace(" ", "_");

        int value = processMessage(username, "getTotalConsumed " + user);

        return value;
    }

    public static int getTotalDrinksConsumed(String username, String drinkType) {
        String user = username.replace(" ", "_");
        String drink = drinkType.replace(" ", "_");

        int value = processMessage(username, "getTotalConsumed " + user + " " + drink);

        return value;
    }

    public static int getTotalDrinksOrdered(String username) {
        String user = username.replace(" ", "_");

        int value = processMessage(username, "getTotalOrdered " + user);

        return value;
    }

    public static int getTotalDrinksOrdered(String username, String drinkType) {
        String user = username.replace(" ", "_");
        String drink = drinkType.replace(" ", "_");

        int value = processMessage(username, "getTotalOrdered " + user + " " + drink);

        return value;
    }

    public static int getTotalDrinksUnconsumed(String username) {
        String user = username.replace(" ", "_");

        int value = processMessage(username, "getTotalUnconsumed " + user);

        return value;
    }

    public static int getTotalDrinksUnconsumed(String username, String drinkType) {
        String user = username.replace(" ", "_");
        String drink = drinkType.replace(" ", "_");

        int value = processMessage(username, "getTotalUnconsumed " + user + " " + drinkType);

        return value;
    }

    public static int getTotalIncorrectClaims(String username) {
        String user = username.replace(" ", "_");

        int value = processMessage(username, "getTotalIncorrectClaims " + user);

        return value;
    }

    public static int getTotalIncorrectClaims(String username, String drinkType) {
        String user = username.replace(" ", "_");
        String drink = drinkType.replace(" ", "_");

        int value = processMessage(username, "getTotalIncorrectClaims " + user + " " + drink);

        return value;
    }

    public static int getTotalRejected(String username) {
        String user = username.replace(" ", "_");

        int value = processMessage(username, "getTotalRejected " + user);

        return value;
    }

    public static int getTotalRejected(String username, String drinkType) {
        String user = username.replace(" ", "_");
        String drink = drinkType.replace(" ", "_");

        int value = processMessage(username, "getTotalRejected " + user + " " + drink);

        return value;
    }

    private static int processMessage(String username, String outputMessage) {
        int value = 0;
        try {
            Socket socket = new Socket(SERVER_IP, SERVER_PORT);
            PrintWriter output = new PrintWriter(socket.getOutputStream(), true);
            Scanner input = new Scanner(socket.getInputStream());

            output.println(outputMessage);
            output.flush();

            String response = input.nextLine();
            value = Integer.parseInt(response);

            output.close();
            input.close();
            socket.close();

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (NumberFormatException e) {
            e.printStackTrace();
        }

        return value;
    }
}
