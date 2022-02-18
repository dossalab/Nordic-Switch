# Nordic-switch (version 1.1 NOT TESTED - See master branch for the current version)

**Before building, read this carefully (especially Safety guidelines)**

This project aims to create a remote-controlled, *smart* light switch. The part presented here is the board that is embedded into the cord switch casing. 
It seems to work, however barely tested in the field (**TL;DR it's quite dangerous and probably can combust spontaneously**).
Software was never fully developed, but you can take any example from Nordic SDK or even use Arduino.

The power supply is rated at 220V / 50 Hz. That's what I had at the moment. If the frequency or/and voltage is different you most likely will need to recalculate the supply.

**Make sure you know what you're doing. It goes without saying that I'm not taking any responsibility for your actions. This device is not for beginners for sure.**

![board render](/images/board.png)

## Bringup

There are 4 solder pads underneath the board. Solder some thin wires to it (thicc ones will most likely rip the pads off the board). **NEVER INTERFACE WITH THE DEVICE WHILE PLUGGED INTO MAINS!** Inject the voltage from separate power supply while testing! Make sure your software runs well *before* testing the device plugged. In the software you need to ensure that:
- The Nordic SoftDevice is not using the LF crystal (since there is no any). **Nothing will work without this step!**;
- All power saving options possibly activated;
- The triac control pin is inverted (0 is active, 1 is not active).

## Some safety guidelines

- **NEVER NEVER** connect the switch to the computer while plugged into mains - you will have a 50 / 50 chance of burning everything. Use low-power 3.3 volt adapter while developing;
- Never touch any parts of the board while operational - you'd likely get shock;
- Never leave device unattended - it's not stable enough to trust it;
- Be generally in a good state of mind and follow what you are doing - **it is easy to make mistakes and burn your stuff**

## TODO

- Proper casing (I just used the one from the real switch, with drilled-out insides;
- **Tests Tests Tests!** Make sure device can work reliably and it is rated for conditions of the power mains.

## Further discussion

(I wrote some really silly description for version 1.0 at https://cringe.page/content/switch/switch.html)

If you assembled device and it worked / not worked for you - please share your stories
