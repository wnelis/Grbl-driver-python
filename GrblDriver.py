#
# GrblDriver.py
#
# This module defines classes to handle the communication to a Grbl (version
# 1.1) equipped controller via a serial port. This driver is a wrapper around
# module serial. It implements the handshake, that is the flow control
# mechanism. Using this wrapper, the calling program experiences a full duplex
# communications channel without flow control issues.
#
# The data streamed to a Grbl device consists of g-code and two groups of
# commands, the system commands (which start with a '$') and the real-time
# control commands. The g-code and the system commands are controlled by one
# flow control mechanism, which is either the so-called send-response mechanism
# or the character-count mechanism. The former has effectively a transmission
# window of one frame, while the latter tries to fill up the receive buffer of
# the Grbl device as much as possible without causing an overflow.
#
# The real-time commands can in principle be sent at anytime, no matter what
# Grbl is doing. However multiple commands will be ignored until it has executed
# the first received command. There are two real-time control commands which
# generate a response. For these two commands a separate send-response flow
# control mechanism is used to prevent loss of commands and thus the problem of
# the flow control getting out of synchronization within this module.
#
# The basic methods are read and write. Those methods rely on three threads, one
# performing input from the serial port and two performing output to the serial
# port. Each flow control mechanism has its own output thread. Each thread can
# thus block whenever necessary without a negative impact on the other threads.
# Together they maintain the flow control state.
#
# Note: the state maintained in this module reflects the state of the
#       communications channel, not necessarily the state of the machine
#       controlled through grbl.
#
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2020.02
#
# Modified by W.J.M. Nelis, wim.nelis@ziggo.nl, 2020.02, v0.03
# - Add send-response flow control (fc-sr) for real-time command '?' and a
#   separate output thread to transfer the real-time control commands. The items
#   with respect to the flow control of the g-codes and the system commands have
#   'gsc' in their name, the items for flow control of real-time control
#   commands have 'rtc' in their name.
#
# Modified by W.J.M. Nelis, wim.nelis@ziggo.nl, 2020.03, v0.04
# - Add a rudimentary form of flow control for the real-time control commands
#   which do not generate a response. A timer is used to force a waiting time of
#   at least 25 [ms] before the next real-time control command can be sent.
#
# To do:
# - Define the commands which imply writing to EEPROM. If in fc-cc, switch to
#   fc-sr. When to switch back?
# - EEPROM commands are '^\$RST=.*$', '^G10 L20? P\d+$', '^G28\.1', '^G30\.1',
#   '^\$\d+=.+$', '^\$I', '^\$N[01]=.+$'
# - Check periodically the status of internal threads (is_alive)
# - Handle the $C system command. It performs a soft reset when check mode is
#   disabled.
# - System commands may only be sent if Grbl is in IDLE mode. See error code 8.
#   Is a system command buffered while Grbl is in non-IDLE mode? Is fc-cc
#   permissible for system commands? How to maintain the mode of Grbl with a
#   (very) low overhead?
#
import queue				# Inter thread communication
import re				# Regular expressions
import serial				# OSI layer 1 protocol
import syslog				# Logging of errors
import threading			# Threads and events
import time				# Timing functions
import watchdog				# Watchdog timer

#
# Configuration parameters.
# -------------------------
#
__version__= '0.04'			# Script version number
gdEnableSyslog= True			# Flag: report errors to syslog
gdRxBufferSize=  128			# Size of receive buffer in grbl device
gdWelcomeMsg= re.compile(r'^Grbl.+\]$')	# RE to recognize a welcome message
gdIntraRtcTime= 0.025			# Minimum intra real-time command time [s]

#
# Define some installation constants, describing aspects of grbl.
# ---------------------------------------------------------------
#
grblSerialBaudRate= 115200		# Serial line baud rate
# grblMaxLineLength= 80			# Max line size, from file protocol.h
grblFlowControl= {			# Supported flow control methods
  'send_response'   : 'sr',  'sr' : 'sr',
  'character_count' : 'cc',  'cc' : 'cc'
}

#
# Define the response per command group which signals some form of completion of
# handling the command. This is needed for both flow control mechanisms. Note
# that there are three major groups of commands: g-codes, system commands and
# real-time control commands. Table grblResponse contains only descriptions of
# commands belonging to the first two categories.
#
grblResponse= [				# List, fix order of evaluation
  [ r'^[A-Za-z].*\n', 'gcode', r'^(?:ok|error:.*)$' ],	# Any G-code
# [ r'^\$J=.*\n'   , 'syscmd', r'^(?:ok|error:.*)$' ],	# A jogging code
  [ r'^\$.*\n'     , 'syscmd', r'^(?:ok|error:.*)$' ],	# A system command
  [ r'^$'          , ''      , r'^ok$' ]		# An empty line
]
for row in grblResponse:		# Optimization, compile RE's
  row[0]= re.compile( row[0] )
  row[2]= re.compile( row[2] )

#
# Define the responses for the real-time control commands. A real-time command
# is always a single byte, thus a value in range(256). Most of them do not
# result in any response, but some do. This table is used for two purposes: to
# extract the real-time commands from the stream of bytes and to determine the
# response, which is needed for flow control purposes.
#
grblRealTimeCmd= [ (False,None)  for i in range(256) ]
for i in range( 128, 256 ):
  grblRealTimeCmd[i]= ( True , None )	# Range of real-time commands
#
# Define the exceptions to the above stated rule.
grblRealTimeCmd[0x18]     = ( True, gdWelcomeMsg )		# ^X
grblRealTimeCmd[ord(b'!')]= ( True, None )			# !
grblRealTimeCmd[ord(b'?')]= ( True, re.compile(r'<.+>$') )	# ?
grblRealTimeCmd[ord(b'~')]= ( True, None )			# ~

#
# Define regular expressions to recognize non-queried responses.
#
NonQueried= [
  gdWelcomeMsg,				# Soft reset completed message
  '^ALARM:',				# Alarm message
  '^\[MSG:.*\]$',			# Unsolicited feedback message
  '^>.*:(?:ok|error:.*)$',		# Start up message ($N)
  '^$'					# Empty line
]
for i in range(len(NonQueried)):	# Optimization
  if isinstance( NonQueried[i], str ):
    NonQueried[i]= re.compile( NonQueried[i] )


#
# Define exception SerialPortError, a derivative of exception RunTimeError,
# which is raised if an error is found while handling the serial port.
#
class SerialPortError( RuntimeError ):
  def __init__( self, arg ):
    super().__init__()
    self.args= arg

#
# Define class myQueue, a derivative of the queue.Queue class, which includes a
# method to clear the queue. This class definition is taken from URL
# https://stackoverflow.com/questions/6517953/clear-all-items-from-the-queue
#
class myQueue( queue.Queue ):
  '''A custom queue subclass that provides a :meth:`clear` method.'''

 #
 # Method clear clears all items from the queue in a threadsafe way.
 #
  def clear( self ):
    with self.mutex:
      unfinished = self.unfinished_tasks - len(self.queue)
      if unfinished <= 0:
        if unfinished < 0:
          raise ValueError('task_done() called too many times')
        self.all_tasks_done.notify_all()
      self.unfinished_tasks = unfinished
      self.queue.clear()
      self.not_full.notify_all()

#
# Define class Grbl, a driver for the serial line to a device running grbl which
# handles the flow control.
#
class Grbl():
  '''Driver to read frames from and sent frames onto a serial line, connected to
     a device running grbl v1.1.'''

  def __init__( self ):
    self.name  = 'Drv'			# Name to use in syslog messages
    self.serial= None			# Serial port object instance

    self.reader_thread= None		# Ingress variables
    self.reader_alive = None
    self.iframe       = bytearray()	# Ingress frame
    self.iframe_queue = myQueue()	# Queue for frames read from grbl
    self.iframe_re_gsc= None		# RE to match response of g-code
    self.iframe_re_rtc= None		# RE to match response of rtc

    self.wr_gsc_thread= None		# Egress variables
    self.wr_gsc_alive = None
    self.wr_gsc_flwctl= threading.Event()	# Flow control state g-code
 #
    self.wr_rtc_thread= None
    self.wr_rtc_alive = None
    self.wr_rtc_flwctl= threading.Event()	# Flow control state rtc
    self.wr_rtc_timer = watchdog.WatchdogTimer()
 #
    self.eframe       = None
    self.eframe_type  = None
    self.eframe_qu_gsc= myQueue()	# Queue for g-codes and system commands
    self.eframe_qu_rtc= myQueue()	# Queue for real-time control commands

    self.flwctl_type  = 'sr'		# Default: use send-response flow control
    self.flwctl_frmcnt= 0		# Unacknowledged frame count
    self.flwctl_rxbfr = gdRxBufferSize	# Free space in Rx buffer of Grbl
    self.flwctl_frmotf= []		# List of frames 'on the fly'

  # Preset the serial port statistics.
    self.sps= dict(			# Define serial port statistics save area
      ingress_total_frames  = 0,	# Total ingress frame count
      ingress_total_octets  = 0,	# Total ingress octet count
      ingress_empty_frames  = 0,	# Total number of empty frames
      ingress_err_protocol  = 0,	# Total number of ingress protocol errors
      egress_total_frames   = 0,	# Total egress frame count
      egress_total_octets   = 0,	# Total egress octet count
      egress_gcode_frames   = 0,	# Number of g-code frames sent
      egress_syscmd_frames  = 0,	# Number of system commands sent
      egress_rtcmd_frames   = 0,	# Number of real time commands sent
#     egress_err_long_frame = 0,	# Total number of frames exceeding max
      egress_fc_hold_count  = 0,	# Total number of wait due to flow control
      egress_fc_hold_time   = 0.0,	# Total time waited due to flow control
      start_time = time.time()		# Start of collection of statistics
    )

  # Let flow control block all traffic until the welcome message is received.
    self._init_flow_control_state()

 #
 # Define the public methods.
 # --------------------------

 #
 # Method clear_statistics resets all statistic counters to zero, and sets the
 # time of start of collecting statistics to the current time.
 #
  def clear_statistics( self ):
    for key in self.sps:
      self.sps[key]= 0
    self.sps['start_time']= time.time()

 #
 # Method close terminates the communication with the serial port and clears all
 # internal queues.
 #
  def close( self ):
    self._stop_threads()		# Stop all threads
    if self.serial is not None:
      self.serial.reset_output_buffer()	# Remove data which is on-the-fly
      self.serial.reset_input_buffer()
      self.serial.close()		# Close serial port
    self.iframe_queue.clear()		# Clear internal queues
    self.eframe_qu_gsc.clear()
    self.eframe_qu_rtc.clear()

 #
 # Getter and setter flowcontrol manage the flow control method to use. If an
 # unknown flow control method is specified in the setter, the method is set to
 # the default (save) value, send-response.
 #
  @property
  def flowcontrol( self ):
    return self.flwctl_type

  @flowcontrol.setter
  def flowcontrol( self, new ):
    if new in grblFlowControl:
      self.flwctl_type= grblFlowControl[new]
    else:
      self.flwctl_type= 'sr'		# Set to the default value

 #
 # Method get_statistics returns the collected statistics.
 #
  def get_statistics( self ):
    return self.sps.copy()		# Return a copy of the statistics

 #
 # Method open clears all internal queues and connects to the serial device.
 #
  def open( self, serdev ):
    try:
      self.serial= serial.Serial( port=serdev, baudrate= grblSerialBaudRate )
    except serial.SerialException as e:
      self._log_message( "Could not open port {}: {}".format(self.serial.name,e) )
      self.serial= None
      return None

    self.iframe_queue.clear()		# Just to be sure, clear internal queues
    self.eframe_qu_gsc.clear()
    self.eframe_qu_rtc.clear()
    self._start_threads()		# Start reading and writing

 #
 # Method read retrieves the next line from the queue containing the lines read
 # from the serial port. The format of the lines is changed slightly: the
 # combination of a carriage return and a line feed at the end of a line is
 # replaced by a single line feed, to match the Linux end-of-line.
 #
  def read( self ):
    bfr= self.iframe_queue.get()
    self.iframe_queue.task_done()
    return bfr + '\n'

  def readqs( self ):
    return self.iframe_queue.qsize()

 #
 # Method write writes a set of lines to an intermediate queue, with one line
 # per queued item. The real-time control commands are extracted and moved to a
 # separate queue.
 #
  def write( self, bfr ):
    if len(bfr) == 0:
      return

    lines= bfr.splitlines( keepends=True )
    for line in lines:
      aline= bytearray()
      for i in range(len(line)):
        achar= line[i]
        anord= ord(achar)
        if anord < 256  and  grblRealTimeCmd[anord][0]:
          self.eframe_qu_rtc.put( { 'Frame' : achar.encode(),
              'Type' : 'rtcmd', 'Re' : grblRealTimeCmd[anord][1] } )
        else:
          aline+= achar.encode('ascii')

      if len(aline) > 0:
        for desc in grblResponse:
          if desc[0].match( aline.decode() ):
            self.eframe_qu_gsc.put( { 'Frame' : aline, 'Type' : desc[1], 'Re' : desc[2] } )
            break
        else:
          self.eframe_qu_gsc.put( { 'Frame' : aline, 'Type' : '', 'Re' : None } )

 #
 # Define the methods which are started as separate threads.
 # ---------------------------------------------------------
 #

 #
 # Method reader is started as a separate thread. It retrieves frames (lines)
 # from the serial port and moves them to the queue which is emptied by method
 # read. If a command has been sent and the received line matches the expected
 # response, the flow control state is updated.
 #
  def reader( self ):
    while self.reader_alive:
      if self.serial is None:
        self.reader_alive= False
        raise SerialPortError( 'Serial device has unexpectedly gone.' )

      self.iframe= self.serial.read_until()	# Read a frame
      if not self.reader_alive:
        break
      self.sps['ingress_total_frames']+= 1
      self.sps['ingress_total_octets']+= len( self.iframe )

  # Clean up the line: move to a utf-8 string and remove the end of line
  # characters.
      self.iframe= self.iframe.decode('utf-8').rstrip('\r\n')

  # See if this frame matches the expected output of a real-time control
  # command. If so update the flow control state for the real-time commands.
      if self.iframe_re_rtc is not None:
        if self.iframe_re_rtc.match( self.iframe ):
          self.wr_rtc_flwctl.set()	# Allow next '?'
          self.iframe_re_rtc= None	# Only one successful match allowed
          self._queue_iframe()
          continue

  # See if this frame matches the last line of the expected output of a g-code
  # or a system command. If so update the flow control state.
      if len(self.flwctl_frmotf) > 0:
        self.iframe_re_gsc= self.flwctl_frmotf[0][1]
      if self.iframe_re_gsc is not None:
        if self.iframe_re_gsc.match( self.iframe ):
          if self.iframe_re_gsc is gdWelcomeMsg:	# Hack: unblock rtc
            self.wr_rtc_flwctl.set()			#  if reset completed
          self._check_flow_control_post()
          self.iframe_re_gsc= None	# Only one successful match allowed
#         self._queue_iframe()			# Optimization
#         continue				# Optimization
#     if self.iframe_re_gsc is not None:	# Optimization
        self._queue_iframe()
        continue

  # Handle a non-queried response. It should be one of the non-queried responses
  # defined in the grbl v1.1 manual. If not, report a protocol error.
      for nqr in NonQueried:
        if nqr.match( self.iframe ):
          self._queue_iframe()
          break
      else:
        self._log_message( 'Unexpected frame: >{}<'.format(self.iframe) )
        self.sps['ingress_err_protocol']+= 1
        self._check_flow_control_post()
        self._queue_iframe()

 #
 # Method writer_gsc is started as a separate thread. It retrieves frames
 # (lines) to transmit from queue eframe_qu_gsc, waits for permission to write
 # the frame and then writes it to the serial port.
 #
  def writer_gsc( self ):
    while self.wr_gsc_alive:
      if self.serial is None:
        self.wr_gsc_alive= False
        raise SerialPortError( 'Serial device has unexpectedly gone.' )

  # Retrieve the next frame (line) from the internal, intermediate queue and
  # update the statistics.
      qv= self.eframe_qu_gsc.get()	# Wait for next line to write
      if not self.wr_gsc_alive:
        break
      self.eframe     = qv['Frame']
      self.eframe_type= qv['Type']
      assert self.eframe_type != 'rtcmd', 'Unexpected frame type on gsc queue.'
  #
      self.sps['egress_total_frames']+= 1
      self.sps['egress_total_octets']+= len( self.eframe )
      if len(self.eframe_type) > 0:
        avar= 'egress_{}_frames'.format( self.eframe_type )
        self.sps[avar]+= 1

  # Wait for permission to send this frame.
      self._check_flow_control_pre()

      dt= time.time()
      self.wr_gsc_flwctl.wait()		# Wait for permission
      dt= time.time() - dt
      if dt > 0.001:
        self.sps['egress_fc_hold_count']+= 1
        self.sps['egress_fc_hold_time' ]+= round( 1000*dt )

      self.wr_gsc_flwctl.clear()	# Use permission only once
      self.flwctl_frmotf.append( (len(self.eframe), qv['Re']) )
      if not self.wr_gsc_alive:
        break

      self.serial.write( self.eframe )	# Finally, write frame to serial port
      self.eframe_qu_gsc.task_done()
      self.serial.flush()		# Wait for frame to be written

 #
 # Method writer_rtc is started as a separate thread. It retrieves real-time
 # control commands to transmit from queue eframe_qu_rtc, waits for permission
 # to write the command and then writes it to the serial port.
 #
  def writer_rtc( self ):
    while self.wr_rtc_alive:
      if self.serial is None:
        self.wr_rtc_alive= False
        raise SerialPortError( 'Serial device has unexpectedly gone.' )

  # Retrieve the next real-time control command and update the statistics.
      qv= self.eframe_qu_rtc.get()	# Wait for next rtc
      if not self.wr_rtc_alive:
        break
      assert qv['Type'] == 'rtcmd', 'Unexpected frame type in rtc queue'
  #
      self.sps['egress_total_frames']+= 1
      self.sps['egress_total_octets']+= 1
      self.sps['egress_rtcmd_frames']+= 1

  # Handle the real-time control commands which generate a response.
      eframe= qv['Frame']
      if qv['Re'] is not None:
        if   eframe == 0x18:		# Soft reset
          self._init_flow_control_state()
        elif eframe == b'?':		# Status query
          dt= time.time()
          self.wr_rtc_flwctl.wait()	# Wait for permission
          dt= time.time() - dt
          if dt > 0.001:
            self.sps['egress_fc_hold_count']+= 1
            self.sps['egress_fc_hold_time' ]+= round( 1000*dt )
          self.wr_rtc_flwctl.clear()	# Use permission only once
          self.iframe_re_rtc= qv['Re']
  # Handle the real-time control commands which do not generate a response.
      else:
        dt= time.time()
        self.wr_rtc_flwctl.wait()	# Wait for permission
        dt= time.time() - dt
        if dt > 0.001:
          self.sps['egress_fc_hold_count']+= 1
          self.sps['egress_fc_hold_time' ]+= round( 1000*dt )
        self.wr_rtc_flwctl.clear()	# Use permission only once
        self.wr_rtc_timer.start( gdIntraRtcTime, self._enable_rtc_flow )

      self.serial.write( eframe )	# Finally, write frame to serial port
      self.eframe_qu_rtc.task_done()


 #
 # Define the private methods.
 # ---------------------------

 #
 # Private method _check_flow_control_pre is called upon arrival of a frame. In
 # case of fc_sr nothing is done. In case of fc_cc, it checks if the frame can
 # be sent at this time. If so, event wr_gsc_flwctl is raised.
 #
  def _check_flow_control_pre( self ):
    self.flwctl_frmcnt+= 1
    self.flwctl_rxbfr -= len( self.eframe )
#   if self.flwctl_type == 'sr':	# Send-response flow control
#     pass
    if self.flwctl_type == 'cc':	# Character counting flow control
      if self.flwctl_rxbfr > 0:
        self.wr_gsc_flwctl.set()

 #
 # Private method _check_flow_control_post is called upon acknowledge of a frame
 # sent. In case of fc_sr, the next frame can be sent now. In case of fc_cc, if
 # there is a frame awaiting transmission and the frame fits in the receive
 # buffer of grbl, allow for transmission of that frame.
 #
  def _check_flow_control_post( self ):
    if   self.flwctl_type == 'sr':	# Send-response flow control
      if self.flwctl_frmcnt <= 2:	# May fail after switch from fc_cc to fc_sr
        self.wr_gsc_flwctl.set()
  #
    elif self.flwctl_type == 'cc':	# Character counting flow control
      if self.flwctl_frmcnt > 1:
        if self.flwctl_rxbfr <= 0:
          rxbfr= self.flwctl_rxbfr + self.flwctl_frmotf[0][0]
          if self.flwctl_frmotf[-1][0] < rxbfr:
            self.wr_gsc_flwctl.set()
  #
    self.flwctl_frmcnt-= 1		# Update flow control parameters
    self.flwctl_rxbfr += self.flwctl_frmotf[0][0]
    self.flwctl_frmotf.pop( 0 )

 #
 # Private method _enable_rtc_flow is a call-back routine which, upon expiration
 # of the watchdog timer, enables output of real-time control commands.
 #
  def _enable_rtc_flow( self ):
    self.wr_rtc_flwctl.set()

 #
 # Private method _init_flow_control_state initialises the flow control state in
 # such a way that the transfer of data to the grbl device is blocked until the
 # welcome message is received. The list of frames-on-the-fly is preset with
 # crafted values, which will preset the flow control state upon receipt of the
 # welcome message.
 #
  def _init_flow_control_state( self ):
    self.wr_gsc_flwctl.clear()		# Revoke permission to transfer
    self.wr_rtc_flwctl.clear()
    self.flwctl_frmcnt= 1		# Block fc-sr
    self.flwctl_rxbfr = 0		# Block fc-cc
    self.flwctl_frmotf= [ (gdRxBufferSize,gdWelcomeMsg) ]

 #
 # Private method _log_message writes a message to a syslog file.
 #
  def _log_message( self, Msg ):
    if gdEnableSyslog:
      syslog.openlog( 'GRBL', 0, syslog.LOG_LOCAL6 )
      syslog.syslog ( ' '.join( (self.name,Msg) ) )
      syslog.closelog()

 #
 # Private method _queue_iframe moves the frame read from the serial port to the
 # queue which is read by method read.
 #
  def _queue_iframe( self ):
    self.iframe_queue.put( self.iframe )
    self.iframe= bytearray()

 #
 # Private method _start_threads starts the internal threads.
 #
  def _start_threads( self ):
   # Start the thread which reads frames from the serial port.
    self.reader_alive = True
    self.reader_thread= threading.Thread( target=self.reader, name='rx' )
    self.reader_thread.start()

   # Start the threads which writes frames to the serial port.
    self.wr_gsc_alive = True
    self.wr_gsc_thread= threading.Thread( target=self.writer_gsc, name='txa' )
    self.wr_gsc_thread.start()
    self.wr_rtc_alive = True
    self.wr_rtc_thread= threading.Thread( target=self.writer_rtc, name='txb' )
    self.wr_rtc_thread.start()


  def _stop_threads( self ):
  # Stop the threads which writes frames to the serial port.
    self.wr_rtc_alive= False
    if self.wr_rtc_thread.is_alive():
      self.eframe_qu_rtc.put( None )
      self.wr_rtc_flwctl.set()
      self.wr_rtc_thread.join()
    self.wr_gsc_alive= False
    if self.wr_gsc_thread.is_alive():
      self.eframe_qu_gsc.put( None )
      self.wr_gsc_flwctl.set()
      self.wr_gsc_thread.join()

   # Stop thread reader.
    self.reader_alive= False
    if self.reader_thread.is_alive():
      if hasattr( self.serial, 'cancel_read' ):
        self.serial.cancel_read()
      self.reader_thread.join()
