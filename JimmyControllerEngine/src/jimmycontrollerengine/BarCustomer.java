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
public class BarCustomer {
    
    private String username;
    private int totalAccepted;
    private int totalChanged;
    private int totalConsumed;
    private int totalOrdered;
    private int totalIncorrect;
    private int totalRejected;
    private HashMap<String, Integer> accepted;
    private HashMap<String, Integer> changed;
    private HashMap<String, Integer> consumed;
    private HashMap<String, Integer> ordered;
    private HashMap<String, Integer> incorrect;
    private HashMap<String, Integer> rejected;
    
    BarCustomer(String name){
        username = name;
        totalAccepted = 0;
        totalChanged = 0;
        totalConsumed = 0;
        totalOrdered = 0;
        totalIncorrect = 0;
        totalRejected = 0;
        accepted = new HashMap<String, Integer>();
        changed = new HashMap<String, Integer>();
        consumed = new HashMap<String, Integer>();
        ordered = new HashMap<String, Integer>();
        incorrect = new HashMap<String, Integer>();
        rejected = new HashMap<String, Integer>();
    }
    
    public void accept(String drinkType){
        if(accepted.containsKey(drinkType))
            accepted.put(drinkType, accepted.get(drinkType)+1);
        else
            accepted.put(drinkType, 1);
        totalAccepted++;
    }
    
    public void change(String drinkType){
        if(changed.containsKey(drinkType))
            changed.put(drinkType, changed.get(drinkType)+1);
        else
            changed.put(drinkType, 1);
        totalChanged++;
    }
    
    public void consume(String drinkType){
        if(consumed.containsKey(drinkType))
            consumed.put(drinkType, changed.get(drinkType)+1);
        else
            consumed.put(drinkType, 1);
        totalConsumed++;
    }
    
    public void order(String drinkType){
        if(ordered.containsKey(drinkType))
            ordered.put(drinkType, ordered.get(drinkType)+1);
        else
            ordered.put(drinkType, 1);
        totalOrdered++;
    }
    
    public void incorrect(String drinkType){
        if(incorrect.containsKey(drinkType))
            incorrect.put(drinkType, incorrect.get(drinkType)+1);
        else
            incorrect.put(drinkType, 1);
        totalOrdered++;
    }
    
    public void reject(String drinkType){
        if(rejected.containsKey(drinkType))
            rejected.put(drinkType, rejected.get(drinkType)+1);
        else
            rejected.put(drinkType, 1);
        totalRejected++;
    }
    
    public int getTotalAccepted() {
        return totalAccepted;
    }

    public int getTotalAccepted(String drinkType) {
        if(accepted.containsKey(drinkType))
            return accepted.get(drinkType);
        else
            return 0;
    }

    public int getTotalChanged() {
        return totalChanged;
    }

    public int getTotalChanged(String drinkType) {
        if(changed.containsKey(drinkType))
            return changed.get(drinkType);
        else
            return 0;
    }
    
    public int getTotalConsumed() {
        return totalConsumed;
    }

    public int getTotalConsumed(String drinkType) {
        if(consumed.containsKey(drinkType))
            return consumed.get(drinkType);
        else
            return 0;
    }
    
    public int getTotalOrdered() {
        return totalOrdered;
    }

    public int getTotalOrdered(String drinkType) {
        if(ordered.containsKey(drinkType))
            return ordered.get(drinkType);
        else
            return 0;
    }
    
    public int getTotalUnconsumed(){
        return totalAccepted - totalConsumed;
    }
    
    public int getTotalUnconsumed(String drinkType){
        return getTotalAccepted(drinkType) - getTotalConsumed(drinkType);
    }
    
    public int getTotalIncorrect() {
        return totalIncorrect;
    }

    public int getTotalIncorrect(String drinkType) {
        if(incorrect.containsKey(drinkType))
            return incorrect.get(drinkType);
        else
            return 0;
    }
    
    public int getTotalRejected() {
        return totalRejected;
    }

    public int getTotalRejected(String drinkType) {
        if(rejected.containsKey(drinkType))
            return rejected.get(drinkType);
        else
            return 0;
    }
}
