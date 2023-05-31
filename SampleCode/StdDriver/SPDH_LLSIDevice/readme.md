SPDH_LLSIDevice sample code
===========================
This LLSI device is used to control the LED light strip, and it is connected to the local bus that behind the SPD5 Hub device on DDR5 architecture.
This LLSI device supports build-in mode and synchronous mode to flash LED light strip.
The build-in mode has some kinds of build-in defined GRB data for flashing LED light strip.
User also can use synchronous mode to send user-defined RGB data for flashing LED light strip.
The host can write the MRn registers of LLSI device through I2C or I3C interface to control the LED light strip on LLSI device.


# Pin connection

| Pin name             | Description                                                                                                            |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| SPDH_LSDA            | The SPDH_LSDA pin of LLSI device needs connect to the SDA of local bus behind the SPD5 Hub.                            |
| SPDH_LSCL            | The SPDH_LSCL pin of LLSI device needs connect to the SCL of local bus behind the SPD5 Hub.                            |
| PB.15(LLSI0_OUT_PB15)| The PB.15 pin is assigned to output LLSI control waveform. So the LED light strip Din needs be connected to PB.15 pin. |

# MR registers of LLSI device

## Register Function Description

| Name |	Description                                |
| -----|---------------------------------------------- |
| MR36 |	LLSI Enable                                |
| MR37 |	LLSI Transfer Mode and Frequency Selection |
| MR38 |	LLSI Build-in Mode LED Function Selection  |
| MR39 |	LLSI Pixel Count Selection 0               |
| MR40 |	LLSI Pixel Count Selection 1               |
| MR41 |	LLSI Data Byte Selection 0                 |
| MR42 |	LLSI Data Byte Selection 1                 |
| MR43 |	LLSI Data                                  |
| MR44 |	Color R                                    |
| MR45 |	Color G                                    |
| MR46 |	Color B                                    |
| MR55 |   LED Speed                                   |
| MR56 |   LED Brightness                              |
        
## Bit field of the regsiters

| Bit Field Name | Register             | Description
| -------------- | -------------------- | -----------------
| LLSIEN	     | MR36[0]	            | 0: Stop to flash LED light strip <br> 1: Start to flash LED light strip
| MODESEL	     | MR37[0]	            | 0: Build-in mode (Flash LED light strip by build-in mode defined RGB data) <br> 1: Synchronous mode (Flash LED light strip by host sent RGB data)
| FRESEL	     | MR37[1]	            | 0: One Shot <br> 1: Continuous
| LEDFUNESEL	 | MR38[3:0]	        | Display type selection for build-in mode<br> 1: Static <br> 2: Breathing <br> 3: Strobe <br> 4: Cycling <br> 5: Random <br> 6: Music <br> 7: Wave <br> 8: Spring <br> 13: Water <br> 14: Rainbow <br> 15: Double Strobe <br> Others: Off
| PCNTSEL	     | MR39[7:0]<br>MR40[8] | LED pixels selection
| BYTESEL	     | MR41[7:0]<br>MR42[8] | Data byte selection*
| DATA	         | MR43[7:0]	        | Data*
| RDATA	         | MR44[7:0]	        | Color R
| GDATA	         | MR45[7:0]	        | Color G
| BDATA	         | MR46[7:0]	        | Color B

Note:
    * : 10 LED need 10 * 3 bytes(RGB) = 30 bytes data, max 300 LED need 300 * 3 bytes = 900 bytes


## LLSI device control flow in build-in mode

### One Shot Type
#### Step:
1. MODESEL = 0, FRESEL = 0
2. LEDFUNESEL = 1 ~ 14
3. PCNTSEL = the count of LED light strip
4. RDATA, GDATA, BDATA=user defined
5. LLSIEN = 1
6. LLSIEN = 0 while Flash has stop


### Continuous Type

#### Control Steps

1. MODESEL = 0 , FRESEL = 1
2. LEDFUNESEL = 1 ~ 14
3. PCNTSEL = the count of LED light strip
4. RDATA, GDATA, BDATA=user defined
5. LLSIEN = 1


## LLSI device control flow in synchronous mode

### One Shot Type

1. MODESEL = 1 , FRESEL = 0
2. PCNTSEL = 1 ~ 14
3. BYTESEL = 0 ~ 511, DATA = user defined
4. LLSIEN = 1
5. LLSIEN = 0 while Flash has stop


### Continuous Type

1. MODESEL = 1 , FRESEL = 1
2. PCNTSEL = 1 ~ 14
3. BYTESEL = 0 ~ 511, DATA = user defined
4. LLSIEN = 1


## Slave Address

User can assign the LID value, but the HID default is 111 before device received SETHID CCC command.
The HID value is decided by the SPD5 Hub.

If the HID is 0 and LID is 6, then the host device can do write and read operation with 0x60 address.

User can modify the slave address with below definition in main.c.
#define I3CS1_SA         (0x67) /* 0110 0111b, LID is 1100b, default HID is 111b before received SETHID CCC command */


## LLSI Control with I2C Protocol

### Write Operation
```
----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
----------------------------------------------
|       | MRn register index          |	A |   |
----------------------------------------------	
|       | Data	                      | A | P |
----------------------------------------------
```

### Read Operation
```
----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
----------------------------------------------
|       | MRn register index          |	A |   |
----------------------------------------------
|  Sr	| Slave Address	       |R = 1 |	A |   |
----------------------------------------------
|       | Data	                      | A | P |
----------------------------------------------
```

## LLSI Control with I3C Protocol

I3C master needs send SETAASA CCC to let LLSI device changed to I3C mode.

### Write Operation
```
----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
----------------------------------------------
|       | MRn register index          |	T |   |
----------------------------------------------	
|       | Data	                      | T | P |
----------------------------------------------
```

### Read Operation
```
----------------------------------------------
|START	| Slave Address	       |W = 0 |	A |   |
----------------------------------------------
|       | MRn register index          |	T |   |
----------------------------------------------
|  Sr	| Slave Address	       |R = 1 |	T |   |
----------------------------------------------
|       | Data	                      | T | P |
----------------------------------------------
```








