OVERVIEW

This project uses a number of ESP32s with attached Waterproof Ultrasonic Module JSN-SR04T Distance Measuring Transducer Sensors to measure the water levels of rain water tanks situated around the property and relay the information to a Reciever ESP32.

The Units located on the Water Tanks are the Slave Send Units (Send Unit)

The Main Receiving Unit is called the Master Receiver Unit (MasterRecUnit)

The Tank capacities and other data is displayed on a 2.8" TFT Screen on the Master Unit and also transmits data via a Webserver.

The transmission of data from all Units is via LoRa.

The Master Unit transmits a coded signal. 

The message is not encrypted and therefore can be received by any LoRa reciever in the vicinity of up to about 100km.

However, the coded message received has ID bytes which informs the Slave Send Units when it is it's turn to send the data back.

The Data sent by the Send Unit also has ID bytes, which must match the ID bytes that was previously sent.

The Master Unit then arranges the data for display purposes. 

The Master also broadcasts the Data via a Webserver on the LAN.

The IP address is different for different ESP32's, and so it is displayed on the TFT screen at start up and connecting to the LAN.

Check the TFT Display for the IP address.

To gain access to the LAN, the SSID and Password must be added to the code.  A WiFi manager is available but not used in this project.

DETAILED FUNCTIONS

The MasterRecUnit sends the Web info every 30 seconds and waits for a signal from any one of the Send Units.  At present 3 can be added to the system,.

The unique part of this project is that one of the Tank SlaveSendUnit (named: Header) is Solar Powered and has a small 600ma LiPo battery to power it during the night. The Header Tank Unit or Header powers OFF into the ESP32 Deep Sleep Mode. This Sleep Mode has a maximum sleep time of 3 hours and the ESP32 uses only 20ua during this mode. With the added modules in the Header Unit the power usage is less than 10ma.

When the Send Unit awakes. it sends the following information:

1. The Distance to the water level in 1 x CM intervals..  (example 9 = 0.09mm)
2. The Operating Power (min 2.8v ~ max 3.3v)
3. The Battery Voltage (min 3.2v ~ max 4.2v) Note: This is not the true voltage of the battery during the day but is correct during the night
4. The Solar Voltage (min 0v ~ max 5v)
5. The Temperature inside the box containing the components.
6. Awake or Sleep Mode as requested by the MasterRecUnit.

The Header Unit is programmed to go into Sleep Mode for the period calculated by the MasterRecUnit. This Sleep Time can be any thing to Zero (Awake Mode) up to 3 hours.  All Sleep Times will calculated to give output on the Hour or at least from the whole hour in periods requested.  Example: For 20m periods the output will be every 20m from the Hour.

If the Header unit's battery voltage drops below 3.3, the unit is put into Sleep Mode for 1 hour and will continue to set the Sleep Mode every hour until the voltage is higher than 3.3 volts.

The Header Unit also automatically goes into a Sleep Mode cycle every 1 hour if the MasterRecUnit is switched off or goes off-line for any reason. 

Upon request for data from the MasterRecUnit to the other 2 SlaveSendUnit(s) named DBL Main and uDeck, the only data sent back is the distance from the top of the tank to the water level. These units will be powered by mains power through a 5v power pack and will operate 24/7.

Of course these additional units can be Solar powered if required.
