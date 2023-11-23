
/*

=0: 9600
=1: 57600
=2: 115200
=3: 230400
=4: 460800
=5: 921600
=6: 1000000
=7: 2000000
=8: 3000000

*/

#define ATC_BAUDRATE "AT+BAUD=3\r"


/*

=0: all off
=1: lcd display on
=2: usb display on
=3: lcd and usb display on
=4: uart display on
=5: lcd and uart display on
=6: usb and uart display on
=7: lcd, usb and uart display on

*/

#define ATC_DISP "AT+DISP=5\r"


// FPS

#define ATC_FPS "AT+FPS=2\r"


/*

1 "1x1 BINN" 	1x1 is equivalent to no binning, and the actual output resolution is 100x100.
2 "2x2 BINN" 	2×2 binning, 4 pixels are merged into 1, the actual output resolution is 50×50. The module ISP is planned to be activated, and the actual output needs to wait for 1 to 2 seconds.
4 "4x4 BINN" 	4×4 binning, 16 pixels are merged into one, and the actual output resolution is 25×25.

*/

#define ATC_BINNMODE "AT+BINN=1\r"


#define ATC_UNIT "AT+UNIT=7\r"

#define FIFO_FULL 120