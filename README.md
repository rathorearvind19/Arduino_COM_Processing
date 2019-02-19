# Arduino_COM_Processing
Integrates Arduino with COM port and processing (language) to communicate with a testchip with custom SPI interface/protocal

Consists of an Arduino Sketch used to control a Arduino Due Board. A Processing script to interface between an input file and the Arduino Sketch. 

File/Folder: Arduino_Sketch
  Containts a set of functions used to configure a testchip [1-5] via a custom SPI interface. Moreover, Arduino reads a plaintext/key from the COM port written by the processing script and sends the data to the testchip via SPI, reads the encrypted  ciphertext (ciphertext generated from the testchip) and writes it back to COM port to be accessed by the processing script. Contains large number of functions inclding string to int covnersion, custom SPI, bit/Byte transfer. 
  
File/Folder: Processing_Sketch
  Contains a set of functions used to read the test vectors (plaintext/key) from a file, writes to the COM port which is polled by the Arduino sketch and reads back the output written on the COM port by the Arduino sketch. The read output is written to an output file. 

Requirements:
  1. Arduino Due Board
  2. Arduino IDE with Board config set to Arduino SAM Board version 1.6.6 on the board manager option under "Tools" tab. 
  3. Processing 3.0
  
  
Please refer to these papers for more details on the measurement setup. 

[1] A. Singh, M. Kar, S. Mathew, A. Rajan, V. De, S. Mukhopadhyay, "25.3 A 128-bit AES Engine with Higher Resistance to Power & Electromagnetic Side-Channel Attacks Enabled by a Security-Aware Integrated All-Digital Low Dropout Regulator", IEEE International Solid-State Circuits Conference (ISSCC), 2019. 
[2] M. Kar, A. Singh, S. K. Mathew, A. Rajan, V. De and S. Mukhopadhyay, "Reducing Power Side-Channel Information Leakage of AES Engines Using Fully Integrated Inductive Voltage Regulator," in IEEE Journal of Solid-State Circuits, vol. 53, no. 8, pp. 2399-2414, Aug. 2018.
[3] M. Kar, A. Singh, S. Mathew, A. Rajan, V. De and S. Mukhopadhyay, "8.1 Improved power-side-channel-attack resistance of an AES-128 core via a security-aware integrated buck voltage regulator," 2017 IEEE International Solid-State Circuits Conference (ISSCC), San Francisco, CA, 2017, pp. 142-143.
[4] A. Singh, M. Kar, S. K. Mathew, A. Rajan, V. De and S. Mukhopadhyay, "Improved Power/EM Side-Channel Attack Resistance of 128-Bit AES Engines With Random Fast Voltage Dithering," in IEEE Journal of Solid-State Circuits.
[5] A. Singh, M. Kar, S. Mathew, A. Rajan, V. De and S. Mukhopadhyay, "Improved power side channel attack resistance of a 128-bit AES engine with random fast voltage dithering," ESSCIRC 2017 - 43rd IEEE European Solid State Circuits Conference, Leuven, 2017, pp. 51-54.
  
