# GD32V003-TIMER2-ENCODER-MODE-example-in-RISCV-assembly

Assembled with BRONZEBEARD assembler. Get bronzebeard assembler from https://github.com/theandrew168/bronzebeard WINDOWS To create a virtual environment, decide upon a directory where you want to place it, and run the venv module as a script with the directory path: python3 -m venv tutorial-env This will create the tutorial-env directory if it doesn’t exist, and also create directories inside it containing a copy of the Python interpreter and various supporting files. A common directory location for a virtual environment is .venv. This name keeps the directory typically hidden in your shell and thus out of the way while giving it a name that explains why the directory exists. It also prevents clashing with .env environment variable definition files that some tooling supports. Once you’ve created a virtual environment, you may activate it.

On Windows, run:

tutorial1-env\Scripts\activate.bat

if test2.asm and CH32V003_reg1.asm (inc files) are on desktop ( both files should be in the same folder) use command

bronzebeard --hex-offset 0x08000000 Desktop/CH32V003_ENCODERmode1_ver1.asm

Use WCH_LINK_UTILITY to upload the assembled bb.out.hex file to CH32V003 chip by connceting gnd, 3.3v,DIO pins. connect TX of usb/serial dongle to PD6 and RX of usb/serial dongle to PD5 0f chip. open terraterm or any other console and set baud to 9600. connect the encoder A & B signals to  PD4 (CH1) and PD3(CH2) and share grounds. count values will be displayed on terminal. If wrong direction swap signal wires. loaded auto reload register with 0xFFFF , so counts from 0 to 65535 and then warps. ATRLR holds the max count. No interrupts counts by hardware. Read the count value by poll. I have added a delay of 1 second between UART update. ENCODER mode1 , mode2 , mode3 examples uploaded
