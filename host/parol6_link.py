"""
parol6_link.py — Talk to the PAROL6 controller from the PC
==========================================================

Two ways to use this file:

1. As a terminal (type commands, see replies):
       python parol6_link.py
       python parol6_link.py COM7               (if it can't find the board)
       python parol6_link.py --port COM7        (same thing, longer form)
   Type HELP for the list of commands, or "quit" to exit.

2. As a library in your own scripts (tests, tuning, later the kinematics):
       from parol6_link import Parol6Link
       arm = Parol6Link()            # finds and opens the board
       arm.move(1, 90)               # waits until the move is DONE
       print(arm.status(1))          # {'pos': 90.0, 'moving': 0, ...}
       print(arm.info(1))            # fixed range and limits from joints.c

   Angles and speeds are for the JOINT (after the 6.4:1 belt on joint 1),
   and MOVE is ABSOLUTE: 0 = standby, or wherever ZERO was last sent.

Built-in bench test:
       python parol6_link.py --test-steps
   Repeats the old power-up step-count check: sends exactly 12,345 steps and
   asks the motor driver how many it actually received.

Needs the pyserial library:   pip install pyserial

HOW THE CONVERSATION WORKS (same rules as command.h in the firmware)
   - We send one command per line.
   - Every command gets exactly ONE reply line: "OK ..." or "ERR <reason>".
   - When a move ends, the controller sends "DONE <joint> pos=<deg>" by itself,
     whenever that happens — so it can arrive between other replies.
   - Lines starting with "#" are messages for a human.
   Because DONE lines can arrive at any time, a background "reader" thread
   reads every line as it comes in and sorts it into the right pile:
   replies go to one queue, DONE lines to another, "#" lines get printed.
"""

import argparse
import queue
import sys
import threading
import time

try:
    import serial                      # pyserial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is not installed. Run:  pip install pyserial")


BAUD = 115200          # must match USART2 in CubeMX
TMC_MSCNT = 0x6A       # driver register: microstep counter (0..1023, wraps)


class ControllerError(Exception):
    """The controller replied ERR, or didn't reply in time."""


def find_board_port():
    """Find the Nucleo's COM port.

    The Nucleo's ST-LINK chip shows up as a USB serial port made by
    STMicroelectronics (USB vendor number 0x0483). Returns its name, e.g.
    'COM5' on Windows, or None if no such port is plugged in.
    """
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 or "STLink" in (p.description or ""):
            return p.device
    return None


class Parol6Link:
    def __init__(self, port=None, echo_info=True):
        port = port or find_board_port()
        if port is None:
            raise ControllerError(
                "Couldn't find the Nucleo. Is the USB cable plugged in? "
                "You can also name the port: Parol6Link('COM5')")
        self.port_name = port
        # timeout=0.1: a read gives up after 0.1 s if nothing arrives, so
        # the reader thread can notice when we want it to stop.
        self._ser = serial.Serial(port, BAUD, timeout=0.1)
        self._echo_info = echo_info
        self._replies = queue.Queue()     # OK / ERR lines
        self._done = queue.Queue()        # (joint, pos) from DONE lines
        self._lock = threading.Lock()     # one command at a time
        self._running = True
        self._bytes_received = 0          # for the "no reply" diagnosis
        self._last_lines = []             # last few lines seen, any kind
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        # Get in step with the controller: an empty line finishes off any
        # half-typed command left in its buffer, then PING checks it answers.
        self._ser.write(b"\n")
        time.sleep(0.05)
        try:
            self.send("PING")
        except ControllerError:
            self.close()
            raise ControllerError(self._explain_no_reply(port)) from None

    def _explain_no_reply(self, port):
        """Turn a silent first PING into a message that says what to check.

        The port opened, so the USB link to the ST-LINK is fine. What's left
        is whether the STM32 is running firmware that answers."""
        if self._bytes_received == 0:
            seen = "Nothing at all came back."
        else:
            seen = (f"{self._bytes_received} bytes came back, but no OK/ERR reply. "
                    f"Last lines: {self._last_lines}")
        return (
            f"{port} opened, but the controller didn't answer PING. {seen}\n"
            "Usual causes, most likely first:\n"
            "  1. The board is running older firmware without serial commands.\n"
            "     In CubeIDE: right-click the project > Refresh (F5) so it sees\n"
            "     serial.c and command.c, then build and flash again.\n"
            "  2. The debugger has the chip paused (Debug mode stops at main()).\n"
            "     Press Resume (F8), or press the black reset button.\n"
            "  3. Wrong port: try  python -m serial.tools.list_ports -v\n"
            "     and look for 'STMicroelectronics STLink Virtual COM Port'.")

    # ---- the background reader ---------------------------------------
    def _read_loop(self):
        while self._running:
            raw = self._ser.readline()          # one line, or b"" on timeout
            if not raw:
                continue
            self._bytes_received += len(raw)
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            self._last_lines = (self._last_lines + [line])[-3:]
            if line.startswith("OK") or line.startswith("ERR"):
                self._replies.put(line)
            elif line.startswith("DONE"):
                # "DONE 1 pos=90.000" -> (1, 90.0)
                parts = line.split()
                joint = int(parts[1])
                pos = float(parts[2].split("=")[1])
                self._done.put((joint, pos))
                if self._echo_info:
                    print(line)
            else:                               # "#" lines and anything odd
                if self._echo_info:
                    print(line)

    # ---- sending commands --------------------------------------------
    def send(self, command, timeout=2.0):
        """Send one command and return its reply line (the text after OK).

        Raises ControllerError if the reply is ERR or none arrives in time.
        """
        with self._lock:
            # Throw away any reply left over from earlier, so we can't
            # mistake it for the answer to this command.
            while not self._replies.empty():
                self._replies.get_nowait()
            self._ser.write((command.strip() + "\n").encode("ascii"))
            try:
                reply = self._replies.get(timeout=timeout)
            except queue.Empty:
                raise ControllerError(f"no reply to '{command}' within {timeout} s")
        if reply.startswith("ERR"):
            raise ControllerError(f"'{command}' -> {reply}")
        return reply[2:].strip()     # text after "OK"

    def wait_done(self, joint, timeout=30.0):
        """Wait for the DONE line of `joint`; returns the final angle."""
        end = time.time() + timeout
        while True:
            left = end - time.time()
            if left <= 0:
                raise ControllerError(f"joint {joint}: no DONE within {timeout} s")
            try:
                j, pos = self._done.get(timeout=left)
            except queue.Empty:
                continue
            if j == joint:
                return pos
            # A DONE for another joint: not ours. (Fine while there's one
            # joint; with several joints this will need a pile per joint.)

    def _clear_done(self):
        while not self._done.empty():
            self._done.get_nowait()

    # ---- convenient wrappers -----------------------------------------
    def move(self, joint, deg, rpm=None, wait=True):
        """Go to an absolute angle. Waits for DONE unless wait=False."""
        self._clear_done()
        cmd = f"MOVE {joint} {deg}" + (f" {rpm}" if rpm else "")
        self.send(cmd)
        return self.wait_done(joint) if wait else None

    def steps(self, joint, n, rpm=None, wait=True):
        """Move by n microsteps. Waits for DONE unless wait=False."""
        self._clear_done()
        self.send(f"STEPS {joint} {n}" + (f" {rpm}" if rpm else ""))
        return self.wait_done(joint) if wait else None

    def status(self, joint):
        """Returns e.g. {'pos': 90.0, 'moving': 0, 'left': 0, 'speed': 30.0,
        'accel': 120.0}"""
        text = self.send(f"STATUS {joint}")       # "1 pos=90.000 moving=0 ..."
        result = {}
        for word in text.split()[1:]:
            key, value = word.split("=")
            result[key] = float(value) if "." in value else int(value)
        return result

    def info(self, joint):
        """The joint's fixed settings, e.g. {'min': -123.047, 'max': 123.047,
        'gear': 6.4, 'max_rpm': 9.375, 'max_accel': 93.75}"""
        text = self.send(f"INFO {joint}")
        return {k: float(v) for k, v in (w.split("=") for w in text.split()[1:])}

    def read_reg(self, joint, reg):
        text = self.send(f"RREG {joint} 0x{reg:02X}")   # "1 reg=0x6A val=0x..."
        return int(text.split("val=")[1], 16)

    def write_reg(self, joint, reg, value):
        self.send(f"WREG {joint} 0x{reg:02X} 0x{value:08X}")

    def stop(self, joint=None):
        self.send("STOP" + (f" {joint}" if joint else ""))

    def close(self):
        self._running = False
        self._reader.join(timeout=1)
        self._ser.close()


# ======================================================================
# Bench test: step count, checked against the motor driver itself
# ======================================================================
def test_step_count(arm, joint=1, n=12345, runs=5):
    """Send exactly n steps and ask the driver how many it received.

    The driver's microstep counter (MSCNT) runs 0..1023 and then wraps round,
    so we can only compare the remainder after dividing by 1024:
        12345 = 12 x 1024 + 57   ->   expect a change of 57,
    or 1024 - 57 = 967 (0x3C7) if the counter runs backwards for this
    direction — which it does with the current wiring (log, 21 Oct 2026).
    Any other number means steps were lost or extra ones crept in.
    """
    expect = {n % 1024, (-n) % 1024}
    all_ok = True
    for run in range(1, runs + 1):
        m0 = arm.read_reg(joint, TMC_MSCNT) & 0x3FF
        arm.steps(joint, n)               # joint's default speed
        time.sleep(0.05)
        m1 = arm.read_reg(joint, TMC_MSCNT) & 0x3FF
        d = (m1 - m0) % 1024
        ok = d in expect
        all_ok &= ok
        print(f"run {run}: driver counted change {d} (0x{d:03X})  "
              f"{'OK' if ok else 'WRONG - expected ' + ' or '.join(map(str, sorted(expect)))}")
        arm.steps(joint, -n)              # back to where we started
    print("STEP COUNT TEST:", "PASSED" if all_ok else "FAILED")
    return all_ok


# ======================================================================
# Terminal mode
# ======================================================================
def terminal(arm):
    print(f"Connected on {arm.port_name}. Type HELP for commands, 'quit' to exit.")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() in ("quit", "exit"):
            break
        if not line:
            continue
        try:
            reply = arm.send(line)
            print("OK", reply)
        except ControllerError as e:
            print(e)


def main():
    ap = argparse.ArgumentParser(description="PAROL6 controller link")
    ap.add_argument("port_pos", nargs="?", metavar="PORT",
                    help="serial port, e.g. COM7 (found automatically if left out)")
    ap.add_argument("--port", help="same as PORT, longer form")
    ap.add_argument("--test-steps", action="store_true", help="run the step-count bench test")
    args = ap.parse_args()

    port = args.port or args.port_pos
    try:
        arm = Parol6Link(port)
    except ControllerError as e:
        # Print the explanation on its own, without a long Python traceback.
        sys.exit(f"ERROR: {e}")
    try:
        if args.test_steps:
            test_step_count(arm)
        else:
            terminal(arm)
    finally:
        # Leave the joint still, whatever happened.
        try:
            arm.stop()
        except ControllerError:
            pass
        arm.close()


if __name__ == "__main__":
    main()
