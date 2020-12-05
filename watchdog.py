#
# WatchdogTimer.py
#
# This module contains the class definition of a simple watchdog timer. The
# timer can be started, stopped and reset. If the timer expires, the supplied
# call back function is invoked.
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2019.01
#
# Note: a timer is a separate thread. After invoking method cancel, the timer is
#   stopped (timer.finished.is_set() == True) but the timer thread may still be
#   alive (timer.is_alive() == True). The time between invoking cancel and the
#   timer thread becoming inactive should be small. To make sure that method
#   is_alive will never return True while the timer is stopped, each call to
#   method cancel is followed by a call to method join.
#
# Note: Method Threading.Timer.run() invokes the user call-back function upon
#   expiration of the timer. Upon invocation of the user call-back function,
#   event timer.finished in not yet set, it will be set upon return. See URL
#   https://github.com/python/cpython/blob/master/Lib/threading.py. Experience
#   shows that if a join is attempted while the timer has expired, exception
#   RunTimeError might be raised with message "cannot join current thread". Thus
#   this problem might occur in method WatchdogTimer.stop. It is locally solved
#   with two modifications: (A) by intercepting the call-back and call
#   timer.finished.set() before the user call-back function is invoked and (B)
#   by extending method WatchdogTimer.is_alive() to return value False if the
#   timer has expired.
#   It is probably better to set event timer.finished in module threading.py
#   (see aforementioned URL) just before invoking the call-back function.
#
from threading import Timer

#
# Define a simple watchdog timer. A specific exception subclass is defined to
# handle a time-out for which no handler is defined.
#
class WdtTimeoutException( Exception ):
  '''An unhandled time-out of a watchdog timer.'''

class WatchdogTimer:
  '''A simple, stoppable and restartable watchdog timer.'''
  def __init__( self, to=None, cb=None ):
    self.timeout= to                    # Time out value [s]
    self.handler= cb                    # Call back function, parameter-less
    self.timer  = Timer( 0, True )      # Dummy timer object instance

 #
 # Private method and call-back function _handler handles an expiration of the
 # timer. The internal timer state is updated, and the user supplied call-back
 # function is invoked.
 #
  def _handler( self ):                 # Default time-out handler
    self.timer.finished.set()           # Timer has stopped
    if self.handler is None:
      raise WdtTimeoutException
    else:
      self.handler()                    # Invoke user handler

 #
 # Private method _start contains the common part of methods start and reset. It
 # stops a timer if there is an active one and starts a new timer using the
 # parameters saved in the object.
 #
  def _start( self ):
    if self.is_alive():
      self.timer.cancel()               # Stop timer
      self.timer.join()                 # Wait for timer thread to finish

    self.timer= Timer( self.timeout, self._handler )
    self.timer.start()
    return True

 #
 # Method is_alive returns True if the timer is running, False otherwise.
 #
  def is_alive( self ):
    return (not self.timer.finished.is_set())  and  self.timer.is_alive()

 #
 # Method reset stops the timer if it is running, and creates and starts a new
 # one using the parameters passed to the previous invocation of method start.
 # If previously no timer was defined, this method does nothing and returns
 # False, while it returns True if the timer is restarted.
 #
  def reset( self ):                    # Reset a running timer
    if self.timeout is None:
      return False                      # Error: no timer defined
    return self._start()                # Stop if necessary and start a timer

 #
 # Method start starts a new timer. If there was a timer already running, it is
 # stopped without further notice. The returned value is False if no timer is
 # started (because no timeout is specified), otherwise the returned value is
 # True.
 #
  def start( self, Timeout, Handler=None ):     # Start a timer
    if Timeout is None:                 # Check for an illegal value
      return False
    self.timeout= Timeout               # Save parameters
    self.handler= Handler
    return self._start()                # Start a new timer

 #
 # Method stop stops the timer if it is running. It returns True if the timer is
 # stopped, False if the timer already is expired.
 #
  def stop( self ):                     # Stop a timer
    if self.is_alive():
      self.timer.cancel()
      self.timer.join()
      return True
    else:
      return False
