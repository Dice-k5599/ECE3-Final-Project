# ECE3-Final-Project

**Project Roadmap**
Due: ~Start of week 10


Graded on: Testing (successful run), Final Report

Objective:


Final Report: Required 

Notes:
- Serial.prints are near dealth (too slow for real-time action)
  * Delays could break the code too
 
- Use bump and user switches (from pwm section from lab 4)
  
- Do a self-calibration at the start of each run
  * Set the car on all white near the start
  * Take a series of reading in the setup() function.
  * Accounts for the ambient IR in the room (could differ from room to room)
  * Set the car in the starting postion and let it go
 
- Visible light doesn't count (don't have to account for it)
  * Sunlight contains IR so make adjustment accordingly
 
- Mesasure voltage often
  * using AD2
  * check battery voltage since the values will bounce between high and low. Duty cycle will determine the speed of the wheels.
 
- Where to start with K_p? (What is K_p?)
  * pick a base PWM. 30-50 is good starting point
  * Choose K_p so that a change from zero error to max error causes a change in the base PWM that is ~50% of the PWM
 
- Work on project on hard surface. Make sure it don't break rover.

- There's a known bug that outputs 2500 (black) when sensoring values from IR transmitter. It only appears in one line of log, so make sure when that when you detect a black line (which indicates the end of the path) that detection isn't phantom.
  * Solution: Wait until atleast 2 all black values until we make the rover turn around. Otherwise the rover will turn around midtrack

- Testing:
  * Start from straight line track
    - pdf with testing track will be on bruinlearn

- Debugging
  * Use LEDs to signal something important
  * Always have fresh batteries on hand
  * Check the signs of K_p and K_d
    - K_p:
      1. Set K_d to 0
      2. Place car offset from, but parallel to track
      3. Watch the first turn
    - K_d:
      1. Set K_p to zero
      2. Place the car offset from, but angle towards to track
      3. Watch the first turn
     
Advanced Project Tips

- Encoder: Each wheel has a encoder, and will put out 360 pulses / revolution. Using this information, we could find out how far the car has traveled.
  * There will be tight turns that won't allow you to travel in fast speed.
 
- Use changeWheelSpeeds function that is provided
- There's a WheelCheck.ino example that will check that thw wheel functions correctly. Also uses encoder counts.

- Break down track into small sections so that debugging is easier

- We can store print out variables AFTER the car stops so that we could keep track of voltage and values
