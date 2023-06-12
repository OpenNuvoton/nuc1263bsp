SPDH_LLSIDevice sample code
===========================
This LLSI device is used to control the LED light strip, and it is connected to the local bus that behind the SPD5 Hub device on DDR5 architecture.
This LLSI device supports build-in mode and synchronous mode to flash LED light strip.
The build-in mode has some kinds of build-in defined RGB data for flashing LED light strip.
User also can use synchronous mode to send user-defined RGB data for flashing LED light strip.
The host can write the MRn registers of LLSI device through I2C or I3C interface to control the LED light strip on LLSI device.



# Pin connection

| Pin name               | Description                                                                                                            |
| ---------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| SPDH_LSDA              | The SPDH_LSDA pin of LLSI device needs connect to the SDA of local bus behind the SPD5 Hub.                            |
| SPDH_LSCL              | The SPDH_LSCL pin of LLSI device needs connect to the SCL of local bus behind the SPD5 Hub.                            |
| PB.15 (LLSI0_OUT_PB15) | The PB.15 pin is assigned to output LLSI control waveform. So the LED light strip Din needs be connected to PB.15 pin. |



# MR registers of LLSI device

## Register Function Description
| Name |	Description                                			|
| ---- |------------------------------------------------------- |
| MR36 |	LLSI Enable                                			|
| MR37 |	LLSI Transfer Mode and Frequency Selection			|
| MR38 |	LLSI Build-in Mode LED Function Selection (for Build-in Mode only) |
| MR39 |	LLSI Pixel Count Selection 0               			|
| MR40 |	LLSI Pixel Count Selection 1               			|
| MR41 |	LLSI Data Byte Selection 0 (for Sync. Mode only) 	|	
| MR42 |	LLSI Data Byte Selection 1 (for Sync. Mode only) 	|
| MR43 |	LLSI Data (for Sync. Mode only)                  	|
| MR44 |	Color R (for Build-in Mode only)                 	|
| MR45 |	Color G (for Build-in Mode only)                 	|
| MR46 |	Color B (for Build-in Mode only)                 	|
| MR47 |    Block Write (for Sync. Mode only)        	 	    |
| MR55 |   	LED Speed (for Build-in Mode only)               	|
| MR56 |   	LED Brightness (for Build-in Mode only)          	|
        
## Bit field of the regsiters
| Bit Field Defintion | Register        | Description
| ------------------- | --------------- | -----------------
| LLSIEN[0]    		  | MR36[0]	   		| 0: Stop to flash LED light strip <br> 1: Start to flash LED light strip
| MODESEL[0] 		  | MR37[0]	        | 0: Build-in mode, flash LED by defined RGB data and effect <br> 1: Synchronous mode, flash LED by host sent RGB data
| FRESEL[0]	     	  | MR37[1]	        | 0: One-Shot <br> 1: Continuous
| DIRSEL[0]	     	  | MR37[2]	        | 0: Forward direction <br> 1: Backward direction
| LEDFUNESEL[3:0]     | MR38[3:0]	    | Display type selection for build-in mode<br> 1: Static <br> 2: Breathing <br> 3: Strobe <br> 4: Cycling <br> 5: Random <br> 6: Music <br> 7: Wave <br> 8: Spring <br> 13: Water <br> 14: Rainbow <br> 15: Double Strobe <br> Others: Off <br> *NOTE: For Build-in Mode only
| PCNTSEL[7:0] 	      | MR39[7:0]		| The number of 8-bit LED pixels defined by MR40[0] and MR39[7:0] <br> *NOTE: If PCNTSEL is 10 (LED counts is 10), the display data needs 30 bytes (10 x 3(R-G-B))
| PCNTSEL[8] 	      | MR40[0]			| The number of 8-bit LED pixels defined by MR40[0] and MR39[7:0] <br> *NOTE: If PCNTSEL is 10 (LED counts is 10), the display data needs 30 bytes (10 x 3(R-G-B))
| BYTESEL[7:0]	      | MR41[7:0]		| The 11-bit byte index selection defined by MR42[2:0] and MR41[7:0] <br> *NOTE-1: Each LED pixel uses 3-byte (R-G-B) for display. If BYTESEL is 4, it means that the G-data buffer of 2nd LED pixel will be updated to SYNCDATA. <br> *NOTE-2: For Sync. Mode only
| BYTESEL[10:8]	      | MR42[2:0]		| The 11-bit byte index selection defined by MR42[2:0] and MR41[7:0] <br> *NOTE-1: Each LED pixel uses 3-byte (R-G-B) for display. If BYTESEL is 4, it means that the G-data buffer of 2nd LED pixel will be updated to SYNCDATA. <br> *NOTE-2: For Sync. Mode only
| SYNCDATA	          | MR43[7:0]	    | SYNCDATA will be updated to BYTESEL position of the LLSI data buffer <br> *NOTE-1: Each LED pixel uses 3-byte (R-G-B) for display. If BYTESEL is 4, it means that the G-data buffer of 2nd LED pixel will be updated to SYNCDATA. <br> *NOTE-2: For Sync. Mode only
| RDATA	        	  | MR44[7:0]	    | Color R-data for lighting effects in Static, Breathing, Strobe, Spring and Double Strobe mode <br> *NOTE: For Build-in Mode only
| GDATA	         	  | MR45[7:0]	    | Color G-data for lighting effects in Static, Breathing, Strobe, Spring and Double Strobe mode <br> *NOTE: For Build-in Mode only
| BDATA	         	  | MR46[7:0]	    | Color B-data for lighting effects in Static, Breathing, Strobe, Spring and Double Strobe mode <br> *NOTE: For Build-in Mode only
| BLKWR         	  | MR47[5:0]	    | Block write mode to update LED strip data. Set the first byte (6-bit) for block write size, then block mode, LED counts and all LED data <br> *NOTE: For Sync. Mode only
| SPEED	         	  | MR55[7:0]	    | Adjust the lighting effect speed <br> *NOTE: For Build-in Mode only
| BRIGHTNESS       	  | MR56[7:0]	    | Adjust the lighting effect brightness <br> *NOTE: For Build-in Mode only



# Device Slave Address

Each local device behind the SPD5 Hub device has a unique 4-bit Local Device ID (LID) code and a 3-bit default HID 111b to establish the default 7-bit I2C slave address, I2C Static Address (SA). 
The HID value of slave address will be updated automatically after the local device receives the SETHID CCC command. And the HID value is determined by SPD5 Hub.

User can modify the default I2C slave address with below definition in main.c.
<br>#define I3CS1_SA         (0x67) * 01100111: SA[6:3] LID is 1100b, SA[2:0] HID is default 111b before received SETHID CCC command */

If the LID is 0xc and the HID is 0x2, then the host can do I2C Protocol write and read operation to LLSI local device with 0x62 address.

The LLSI device can be changed to operate in I3C mode after receiving SETAASA CCC command. The 7-bit I2C slave address will become a 7-bit I3C Dynamic Address (DA), 
and the local device will become an I3C slave device.
After that, the host device need to do write and read operation to LLSI device with I3C Protocol.



# LLSI Device Control Example

## Continuous Breathing Effect in Build-in Mode

### Control Steps Description
1. Select build-in mode 
<br>[ Host Write ] MODSEL (MR37[0]) 0
2. Select continuous flash 
<br>[ Host Write ] FRESEL (MR37[1]) 1
3. Select LED breathing effect 
<br>[ Host Write ] LEDFUNESEL (MR38[3:0]) 2
4. Set the number of LED to 10 
<br>[ Host Write ] PCNTSEL[7:0] (MR39[7:0]) 0xa <br>[ Host Write ] PCNTSEL[8] (MR40[0]) 0
5. Set all LED colors to Blue 
<br> [ Host Write ] RDATA (MR44[7:0]) 0x00  <br> [ Host Write ] GDATA (MR45[7:0]) 0x00 <br> [ Host Write ] BDATA (MR46[7:0]) 0xff
6. Start to flash LED strip 
<br>[ Host Write ] LLSIEN (MR36[0]) 1

### Host (Master) Operation Commands
| Address + RnW | Data Paylod | Description
| ------------- | ----------- | -----------
| SA/DA + W=0  	| 0x25-0x02	  | Write MR37 0x02 to select continuous flash in build-in mode
| SA/DA + W=0  	| 0x26-0x02	  | Write MR38 0x02 to select breathing effect
| SA/DA + W=0 	| 0x27-0x0a	  | Write MR39 0x0a to set the number of LED to 10 (PCNTSEL = 10)
| SA/DA + W=0  	| 0x28-0x00	  | Write MR40 0x00 to set the number of LED to 10 (PCNTSEL = 10)
| SA/DA + W=0  	| 0x2c-0x00	  | Write MR44 0x00 (R-data) to set all LED colors to blue
| SA/DA + W=0  	| 0x2d-0x00	  | Write MR45 0x00 (G-data) to set all LED colors to blue
| SA/DA + W=0  	| 0x2e-0xff	  | Write MR46 0xff (B-data) to set all LED colors to blue
| SA/DA + W=0  	| 0x24-0x01	  | Write MR36 0x01 to flash LED strip

## User-defined LED Colors in Synchronous Mode by 1-byte Write

### Control Steps Description
1. Select synchronous mode 
<br>[ Host Write ] MODSEL (MR37[0]) 1
2. Select one-shot flash 
<br>[ Host Write ] FRESEL (MR37[1]) 0
3. Set the number of LED to 3 
<br>[ Host Write ] PCNTSEL[7:0] (MR39[7:0]) 3 <br>[ Host Write ] PCNTSEL[8] (MR40[0]) 0
4. Set 1st LED pixel to Red (R-data:0xff / G-data:0x00 / B-data:0x00) 
<br>[ Host Write ] BYTESEL[7:0] (MR41[7:0]) 0 <br>[ Host Write ] BYTESEL[10:8] (MR42[2:0]) 0 <br>[ Host Write ] SYNCDATA (MR43[7:0]) 0xff (R-data of LED-0)
<br>[ Host Write ] BYTESEL[7:0] (MR41[7:0]) 1 <br>[ Host Write ] BYTESEL[10:8] (MR42[2:0]) 0 <br>[ Host Write ] SYNCDATA (MR43[7:0]) 0x00 (G-data of LED-0)
<br>[ Host Write ] BYTESEL[7:0] (MR41[7:0]) 2 <br>[ Host Write ] BYTESEL[10:8] (MR42[2:0]) 0 <br>[ Host Write ] SYNCDATA (MR43[7:0]) 0x00 (B-data of LED-0)
5. Set 2nd LED pixel to Green (R-data:0x00 / G-data:0xff / B-data:0x00) 
<br>[ Host Write ] BYTESEL[7:0] (MR41[7:0]) 3 <br>[ Host Write ] BYTESEL[10:8] (MR42[2:0]) 0 <br>[ Host Write ] SYNCDATA (MR43[7:0]) 0x00 (R-data of LED-1)
<br>[ Host Write ] BYTESEL[7:0] (MR41[7:0]) 4 <br>[ Host Write ] BYTESEL[10:8] (MR42[2:0]) 0 <br>[ Host Write ] SYNCDATA (MR43[7:0]) 0xff (G-data of LED-1)
<br>[ Host Write ] BYTESEL[7:0] (MR41[7:0]) 5 <br>[ Host Write ] BYTESEL[10:8] (MR42[2:0]) 0 <br>[ Host Write ] SYNCDATA (MR43[7:0]) 0x00 (B-data of LED-1)
6. Set 3rd LED pixel to Blue (R-data:0x00 / G-data:0x00 / B-data:0xff) 
<br>[ Host Write ] BYTESEL[7:0] (MR41[7:0]) 6 <br>[ Host Write ] BYTESEL[10:8] (MR42[2:0]) 0 <br>[ Host Write ] SYNCDATA (MR43[7:0]) 0x00 (R-data of LED-2)
<br>[ Host Write ] BYTESEL[7:0] (MR41[7:0]) 7 <br>[ Host Write ] BYTESEL[10:8] (MR42[2:0]) 0 <br>[ Host Write ] SYNCDATA (MR43[7:0]) 0x00 (G-data of LED-2)
<br>[ Host Write ] BYTESEL[7:0] (MR41[7:0]) 8 <br>[ Host Write ] BYTESEL[10:8] (MR42[2:0]) 0 <br>[ Host Write ] SYNCDATA (MR43[7:0]) 0xff (B-data of LED-2)
7. Start to flash LED strip 
<br>[ Host Write ] LLSIEN (MR36[0]) 1

### Host (Master) Operation Commands
| Address + RnW | Data Paylod | Description
| ------------- | ----------- | -----------
| SA/DA + W=0  	| 0x25-0x01	  | Write MR37 0x01 to select one-shot flash in synchronous mode
| SA/DA + W=0 	| 0x27-0x03	  | Write MR39 0x03 to set the number of LED to 3 (PCNTSEL = 3)
| SA/DA + W=0  	| 0x28-0x00	  | Write MR40 0x00 to set the number of LED to 3 (PCNTSEL = 3)
| SA/DA + W=0 	| 0x29-0x00	  | Write MR41 0x00 to set data index 0 	(BYTESEL = 0, R-data of LED-0)
| SA/DA + W=0  	| 0x2a-0x00	  | Write MR42 0x00 to set data index 0 	(BYTESEL = 0, R-data of LED-0)
| SA/DA + W=0  	| 0x2b-0xff	  | Write MR43 to set R-data 0xff 	        (BYTESEL = 0, R-data of LED-0)
| SA/DA + W=0 	| 0x29-0x01	  | Write MR41 0x01 to set data index 1 	(BYTESEL = 1, G-data of LED-0)
| SA/DA + W=0  	| 0x2a-0x00	  | Write MR42 0x00 to set data index 1 	(BYTESEL = 1, G-data of LED-0)
| SA/DA + W=0  	| 0x2b-0x00	  | Write MR43 to set G-data 0x00	        (BYTESEL = 1, G-data of LED-0)
| SA/DA + W=0 	| 0x29-0x02	  | Write MR41 0x02 to set data index 2 	(BYTESEL = 2, B-data of LED-0)
| SA/DA + W=0  	| 0x2a-0x00	  | Write MR42 0x00 to set data index 2 	(BYTESEL = 2, B-data of LED-0)
| SA/DA + W=0  	| 0x2b-0x00	  | Write MR43 to set B-data 0x00	        (BYTESEL = 2, B-data of LED-0)
| SA/DA + W=0 	| 0x29-0x03	  | Write MR41 0x03 to set data index 3 	(BYTESEL = 3, R-data of LED-1)
| SA/DA + W=0  	| 0x2a-0x00	  | Write MR42 0x00 to set data index 3 	(BYTESEL = 3, R-data of LED-1)
| SA/DA + W=0  	| 0x2b-0x00	  | Write MR43 to set R-data 0x00 	        (BYTESEL = 3, R-data of LED-1)
| SA/DA + W=0 	| 0x29-0x04	  | Write MR41 0x04 to set data index 4 	(BYTESEL = 4, G-data of LED-1)
| SA/DA + W=0  	| 0x2a-0x00	  | Write MR42 0x00 to set data index 4 	(BYTESEL = 4, G-data of LED-1)
| SA/DA + W=0  	| 0x2b-0xff	  | Write MR43 to set G-data 0xff	        (BYTESEL = 4, G-data of LED-1)
| SA/DA + W=0 	| 0x29-0x05	  | Write MR41 0x05 to set data index 5 	(BYTESEL = 5, B-data of LED-1)
| SA/DA + W=0  	| 0x2a-0x00	  | Write MR42 0x00 to set data index 5 	(BYTESEL = 5, B-data of LED-1)
| SA/DA + W=0  	| 0x2b-0x00	  | Write MR43 to set B-data 0x00	        (BYTESEL = 5, B-data of LED-1)
| SA/DA + W=0 	| 0x29-0x06	  | Write MR41 0x06 to set data index 6 	(BYTESEL = 6, R-data of LED-2)
| SA/DA + W=0  	| 0x2a-0x00	  | Write MR42 0x00 to set data index 6 	(BYTESEL = 6, R-data of LED-2)
| SA/DA + W=0  	| 0x2b-0x00	  | Write MR43 to set R-data 0x00 	        (BYTESEL = 6, R-data of LED-2)
| SA/DA + W=0 	| 0x29-0x07	  | Write MR41 0x07 to set data index 7 	(BYTESEL = 7, G-data of LED-2)
| SA/DA + W=0  	| 0x2a-0x00	  | Write MR42 0x00 to set data index 7 	(BYTESEL = 7, G-data of LED-2)
| SA/DA + W=0  	| 0x2b-0x00	  | Write MR43 to set G-data 0x00	        (BYTESEL = 7, G-data of LED-2)
| SA/DA + W=0 	| 0x29-0x08	  | Write MR41 0x08 to set data index 8 	(BYTESEL = 8, B-data of LED-2)
| SA/DA + W=0  	| 0x2a-0x00	  | Write MR42 0x00 to set data index 8 	(BYTESEL = 8, B-data of LED-2)
| SA/DA + W=0  	| 0x2b-0xff	  | Write MR43 to set B-data 0xff           (BYTESEL = 8, B-data of LED-2)
| SA/DA + W=0  	| 0x24-0x01	  | Write MR36 0x01 to flash LED strip


## User-defined LED Colors in Synchronous Mode by Block Write

### Control Steps Description
1. Select synchronous mode 
<br>[ Host Write ] MODSEL (MR37[0]) 1
2. Select one-shot flash 
<br>[ Host Write ] FRESEL (MR37[1]) 0
3. Enter block write mode to set all LED colors. 
<br>Host will send 47 (MR index, 1-byte), 13 (block size, 1-byte), 0 (block mode, 1-byte), 3 (LED pixels, 2-bytes) and sequent 30 bytes (color data) to set LED-0 Red, LED-1 Green, LED-2 Blue.
<br>[ Host Write ] BLKWR (MR47[5:0]) 13, then 0x00-0x00-0x03-0xff-0x00-0x00-0x00-0xff-0x00-0x00-0x00-0xff
4. Start to flash LED strip 
<br>[ Host Write ] LLSIEN (MR36[0]) 1

### Host (Master) Operation Commands
| Address + RnW | Data Paylod | Description
| ------------- | ----------- | -----------
| SA/DA + W=0  	| 0x25-0x01	  | Write MR37 0x01 to select one-shot flash in synchronous mode
| SA/DA + W=0 	| 0x2f-0x0d-0x00-0x00-03-0xff-0x00-0x00-0x00-0xff-0x00-0x00-0x00-0xff | Write MR47 to update LED0&1&2 data
| SA/DA + W=0  	| 0x24-0x01	  | Write MR36 0x01 to flash LED strip



# LLSI Control with I2C/I3C Protocol

## Host Write/Read Control with I2C Protocol
LLSI local device is default at I2C mode.

### Write Operation
```
-----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------
|       | MRn register index          |	A |   |
-----------------------------------------------	
|       | Data	                      | A |   |
-----------------------------------------------	
|       | ...	                      | A |   |
-----------------------------------------------	
|       | Data	                      | A | P |
-----------------------------------------------
```

### Read Operation
```
-----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------
|       | MRn register index          |	A |   |
-----------------------------------------------
|  Sr	| Slave Address	       |R = 1 |	A |   |
-----------------------------------------------
|       | Data	                      | A |   |
-----------------------------------------------	
|       | ...	                      | A |   |
-----------------------------------------------	
|       | Data	                      | A | P |
-----------------------------------------------
```

## Host Write/Read Control with I3C Protocol
LLSI local device will change to I3C mode after receiving SETAASA CCC command.

### Write Operation w/ PEC Disabled
```
----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------
|       | MRn register index          |	T |   |
-----------------------------------------------	
|       | Data	                      | T |   |
-----------------------------------------------	
|       | ...	                      | T |   |
-----------------------------------------------	
|       | Data	                      | T | P |
----------------------------------------------
```

### Write Operation w/ PEC Enabled
```
-----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------
|       | MRn register index          |	T |   |
-----------------------------------------------	
|       | 3-bits CMD | W = 0 | 0000   | T |   |
-----------------------------------------------	
|       | Data	                      | T |   |
-----------------------------------------------	
|       | ...	                      | T |   |
-----------------------------------------------	
|       | Data	                      | T |   |
-----------------------------------------------
|       | PEC	                      | T | P |
-----------------------------------------------
```

### Read Operation w/ PEC Disabled
```
-----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------
|       | MRn register index          |	T |   |
-----------------------------------------------
|  Sr	| Slave Address	       |R = 1 |	T |   |
-----------------------------------------------
|       | Data	                      | T |   |
-----------------------------------------------
|       | ...	                      | T |   |
-----------------------------------------------
|       | Data	                      | T | P |
-----------------------------------------------
```

### Read Operation w/ PEC Enabled
```
-----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
-----------------------------------------------
|       | MRn register index          |	T |   |
-----------------------------------------------	
|       | 3-bits CMD | R = 1 | 0000   | T |   |
-----------------------------------------------
|       | PEC	                      | T |   |
-----------------------------------------------
|  Sr	| Slave Address	       |R = 1 |	T |   |
-----------------------------------------------
|       | Data	                      | T |   |
-----------------------------------------------
|       | ...	                      | T |   |
-----------------------------------------------
|       | Data	                      | T |   |
-----------------------------------------------
|       | PEC	                      | T | P |
-----------------------------------------------
```

## Command Truth Table (CMD) w/ PEC Enabled
```
----------------------------------------------------------------
| Command Description	    | Command Name | Command Code | RW |
----------------------------------------------------------------
| Write 1 Byte to Register  | W1R          | 000          | 0  |
----------------------------------------------------------------
| Read 1 Byte from Register | R1R          | 000          | 1  |
----------------------------------------------------------------
| Write 2 Byte to Register  | W2R          | 001          | 0  |
----------------------------------------------------------------
| Read 2 Byte from Register | R2R          | 001          | 1  |
----------------------------------------------------------------
NOTE: Command Code defined at 2nd byte[7:5], RW defined at 2nd byte[4].    
```

