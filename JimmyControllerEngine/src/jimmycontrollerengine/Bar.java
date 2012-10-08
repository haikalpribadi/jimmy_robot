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

import java.util.HashMap;

/**
 *
 * @author Haikal Pribadi <haikal.pribadi@gmail.com>
 */
public class Bar {

    HashMap<String, BarCustomer> customers;

    Bar() {
        customers = new HashMap<String, BarCustomer>();
    }

    public void accept(String username, String drinkType) {
        if (!customers.containsKey(username)) {
            customers.put(username, new BarCustomer(username));
        }
        customers.get(username).accept(drinkType);
    }

    public void changeDrink(String username, String drinkType) {
        if (!customers.containsKey(username)) {
            customers.put(username, new BarCustomer(username));
        }
        customers.get(username).change(drinkType);
    }

    public void consumeDrink(String username, String drinkType) {
        if (!customers.containsKey(username)) {
            customers.put(username, new BarCustomer(username));
        }
        customers.get(username).consume(drinkType);
    }

    public void orderDrink(String username, String drinkType) {
        if (!customers.containsKey(username)) {
            customers.put(username, new BarCustomer(username));
        }
        customers.get(username).order(drinkType);
    }

    public void incorrectDrink(String username, String drinkType) {
        if (!customers.containsKey(username)) {
            customers.put(username, new BarCustomer(username));
        }
        customers.get(username).incorrect(drinkType);
    }

    public void rejectDrink(String username, String drinkType) {
        if (!customers.containsKey(username)) {
            customers.put(username, new BarCustomer(username));
        }
        customers.get(username).reject(drinkType);
    }

    public int getTotalAccepted(String username) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalAccepted();
        } else {
            return 0;
        }
    }

    public int getTotalAccepted(String username, String drinkType) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalAccepted(drinkType);
        } else {
            return 0;
        }
    }

    public int getTotalChangeRequests(String username) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalChanged();
        } else {
            return 0;
        }
    }

    public int getTotalChangeRequests(String username, String drinkType) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalChanged(drinkType);
        } else {
            return 0;
        }
    }

    public int getTotalDrinksConsumed(String username) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalConsumed();
        } else {
            return 0;
        }
    }

    public int getTotalDrinksConsumed(String username, String drinkType) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalConsumed(drinkType);
        } else {
            return 0;
        }
    }

    public int getTotalDrinksOrdered(String username) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalOrdered();
        } else {
            return 0;
        }
    }

    public int getTotalDrinksOrdered(String username, String drinkType) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalOrdered(drinkType);
        } else {
            return 0;
        }
    }

    public int getTotalDrinksUnconsumed(String username) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalUnconsumed();
        } else {
            return 0;
        }
    }

    public int getTotalDrinksUnconsumed(String username, String drinkType) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalUnconsumed(drinkType);
        } else {
            return 0;
        }
    }

    public int getTotalIncorrectClaims(String username) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalIncorrect();
        } else {
            return 0;
        }
    }

    public int getTotalIncorrectClaims(String username, String drinkType) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalIncorrect(drinkType);
        } else {
            return 0;
        }
    }

    public int getTotalRejected(String username) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalRejected();
        } else {
            return 0;
        }
    }

    public int getTotalRejected(String username, String drinkType) {
        if (customers.containsKey(username)) {
            return customers.get(username).getTotalRejected(drinkType);
        } else {
            return 0;
        }
    }
}
