Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements in the "Default" configuration of the profiler to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See [Shared document](https://docs.google.com/document/d/1Ro9G2Nsr_ZXDhBYJ6YyF9CPivb--6UjhHRmVhDGySag/edit?usp=sharing) for instructions.* 

1. What is the average current per period?
   Answer:
   100uA
   ![Avg_current_per_period](screenshots/assignment4/avg_current_per_period.jpg)  

2. What is the average current when the Si7021 is Load Power Management OFF?
   Answer:
   9uA 
   ![Avg_current_LPM_Off](screenshots/assignment4/avg_current_lpm_off.jpg)  

3. What is the average current when the Si7021 is Load Power Management ON?
   Answer:
   2mA  
   ![Avg_current_LPM_Off](screenshots/assignment4/avg_current_lpm_on.jpg)  

4. How long is the Si7021 Load Power Management ON for 1 temperature reading?
   Answer:
   138ms 
   ![duration_lpm_on](screenshots/assignment4/avg_current_lpm_on.jpg)  

5. What is the total operating time of your design for assignment 4 in hours assuming a 1000mAh supply?
   There is a consumption of 100uA over 3s. Over an hour, which is 120mA over an hour. This means the 1000mAh supply would last
   for 1000/120 = 8.34hours at a stretch.

6. How has the power consumption performance of your design changed since the previous assignment?
                        |           Old                    |           New
     Total Consumption  |          238.43uA                |           100uA
     Based on the consumption in the question above, the battery would last for approximately half the time. That is, for 4hours.
7. Describe how you have tested your code to ensure you are sleeping in EM1 mode during I2C transfers.
   According to the state transitions, once the write to i2c is initiated, interrrupts for I2C are enabled and EM1 is entered manually.
   (using EMU_EnterEM1();). Once, an interrupt occurs, indicating write complete, the state is switched to read and it comes out of the EM1.