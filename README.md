# Grbl-driver-python
Flow control driver for Grbl v1.1

### Introduction

GrblDriver is a python3 module which handles the flow control when communicating
with a Grbl v1.1 device. An application, typically a GUI, sends commands and
expects responses. The commands are buffered within this module until the flow
control state is such that the commands can be sent to the Grbl device.

GrblDriver divides the commands into two groups. The first group contains both
the g-codes and the system commands, while the second group contains the
real-time control commands. These two groups of commands are handled
differently. The commands in the first group are subject to the selected flow
control method, which can be either 'send-response' of 'character counting'. For
the commands in the second group the 'send-response' method is used for those
real-time control commands which generate a response. For the other real-time
control commands, a minimum waiting time between successive commands is used to
prevent loss of control commands.

### Status

The current implementation is a proof of concept. It has not been tested
thoroughly (yet), and more over, it is not yet complete.

There are some fundamental problems with flow control. It is not clear how to
detect that the flow control state within the Grbl device and the flow control
state with the driver are out of sync. Once an out-of-sync is detected, it is
not clear how to regain synchronization. Probably a complete reset (restart) of
both the Grbl device and GrblDriver is necessary.

At least two parts of the flow control still need to be implemented. The first
part is related to the commands which write to EEPROM. From the documentation it
is concluded that in the time the EEPROM is written, no commands should be sent
to the Grbl device. That implies that during that time the flow control method
'character counting' temporarily must be replaced by method 'send-response'. The
second part to implement is related to the check mode, using system command $C.
If the check mode is disabled, a soft reset is performed by the Grbl device and
thus transmission must be suspended until the welcome message is received.

From the description of error 8 (Grbl '$' command cannot be used unless Grbl is
IDLE. Ensures smooth operation during a job), one might conclude that flow
control method 'character counting' is not suitable for system commands, as a
system command might be sent while the state of the Grbl device is not idle.
Using method 'send-response', the change of hitting the idle state seems to be
much higher or maybe even 100%. I do not know if the aforementioned conclusions
are correct.

### Design

The structure of the module is shown in [this picture](docs/grbl.driver.architecture.pdf).

