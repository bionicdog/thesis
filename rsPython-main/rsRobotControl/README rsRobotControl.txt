Voorbeelden:
studentwork/2020Tommy/cameraTommy.py
studentwork/2021Marco/main.py
studentwork/2019Nikolai/main.py

** Here we store control programs for robots **

* Python is running on raspberry pi, connected to arduino with serial connection (aparte thread)

 - data from arduino is gathered in separate thread
 
* met robotControl -> motorControl
 
* sensors uitlezen
	- aparte thread van main control
	- met alarmen

* picam
	- evt in aparte thread (samen met uitlezen sensors)
	
* keyboardcontrol OF automatisch < MAIN THREAD?
	- keyboard via thread
  cf cameraTommy

* evt naar laptop
 
* met algemene dictionary (~Observer > hernoemen naar Monitor)

* met algemene stop (via keyboard OF alarm)