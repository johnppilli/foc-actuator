#!/usr/bin/env python3
"""Telemetry decoder / serial console for the firmware.

Frame (little-endian): AA 55 | len=42 | payload | xor checksum
Payload: "<I9fBB" = tick, theta_e, theta_m, omega_m, i_d, i_q, i_q_ref, v_d, v_q, v_bus, mode, fault
Must match foc/telemetry.h.

    # decode a file the C unit test wrote, to prove both sides agree
    python3 tools/telemetry.py --selftest build/telem_sample.bin

    # record 5 s of telemetry to CSV
    python3 tools/telemetry.py --port /dev/tty.usbmodemXXXX --seconds 5 --csv run.csv

    # send a command (see firmware/Core/Src/board_serial.c for the list)
    python3 tools/telemetry.py --port /dev/tty.usbmodemXXXX --cmd "m torque" --cmd "q 0.3"

    # live plot
    python3 tools/telemetry.py --port /dev/tty.usbmodemXXXX --plot
"""
import argparse
import collections
import struct
import sys
import time

SYNC = b"\xaa\x55"
PAYLOAD_FMT = "<I9fBB"
PAYLOAD_LEN = struct.calcsize(PAYLOAD_FMT)  # 42
FRAME_LEN = 3 + PAYLOAD_LEN + 1
FIELDS = ["tick", "theta_e", "theta_m", "omega_m", "i_d", "i_q", "i_q_ref", "v_d", "v_q", "v_bus", "mode", "fault"]
MODES = {0: "idle", 1: "openloop", 2: "calib", 3: "torque", 4: "haptic", 5: "fault"}


class Decoder:
    """Byte-stream decoder that resynchronises on the sync word."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # keep a trailing 0xAA in case the 0x55 arrives next
                self.buf = self.buf[-1:] if self.buf and self.buf[-1] == 0xAA else bytearray()
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < FRAME_LEN:
                break
            if self.buf[2] != PAYLOAD_LEN:
                del self.buf[:1]
                continue
            payload = bytes(self.buf[3 : 3 + PAYLOAD_LEN])
            x = 0
            for b in payload:
                x ^= b
            if x != self.buf[3 + PAYLOAD_LEN]:
                del self.buf[:1]
                continue
            frames.append(dict(zip(FIELDS, struct.unpack(PAYLOAD_FMT, payload))))
            del self.buf[:FRAME_LEN]
        return frames


def selftest(path):
    with open(path, "rb") as f:
        data = f.read()
    frames = []
    dec = Decoder()
    # feed in awkward chunk sizes to exercise resync
    for i in range(0, len(data), 7):
        frames += dec.feed(data[i : i + 7])
    ok = len(frames) == 5 and all(fr["tick"] == i for i, fr in enumerate(frames))
    ok = ok and all(abs(fr["i_q"] - 0.1 * i) < 1e-6 for i, fr in enumerate(frames))
    ok = ok and frames[0]["v_bus"] == 12.0 and frames[0]["mode"] == 3
    for fr in frames:
        print(fr)
    print("selftest", "OK" if ok else "FAILED")
    return 0 if ok else 1


def open_port(port, baud):
    import serial  # pyserial

    return serial.Serial(port, baud, timeout=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", metavar="FILE")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl-C)")
    ap.add_argument("--csv", help="write decoded frames here")
    ap.add_argument("--cmd", action="append", default=[], help="send this line to the firmware (repeatable)")
    ap.add_argument("--plot", action="store_true", help="live plot")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest(args.selftest))
    if not args.port:
        ap.error("--port or --selftest required")

    ser = open_port(args.port, args.baud)
    for c in args.cmd:
        ser.write((c.strip() + "\n").encode())
        time.sleep(0.05)
    if args.cmd and not (args.csv or args.plot or args.seconds):
        # command-only: echo whatever text the firmware answers with
        time.sleep(0.2)
        sys.stdout.write(ser.read(4096).decode(errors="replace"))
        return

    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w")
        csv_file.write(",".join(FIELDS) + "\n")

    hist = None
    if args.plot:
        import matplotlib.pyplot as plt

        plt.ion()
        n = 2000
        hist = {k: collections.deque(maxlen=n) for k in ("tick", "i_q", "i_q_ref", "i_d", "theta_m", "omega_m")}
        fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        lines = {
            "i_q_ref": ax[0].plot([], [], "k--", label="iq ref")[0],
            "i_q": ax[0].plot([], [], label="iq")[0],
            "i_d": ax[0].plot([], [], label="id")[0],
            "theta_m": ax[1].plot([], [], label="theta_m")[0],
            "omega_m": ax[2].plot([], [], label="omega_m")[0],
        }
        for a in ax:
            a.legend(loc="upper left")
        ax[0].set_ylabel("A")
        ax[1].set_ylabel("rad")
        ax[2].set_ylabel("rad/s")

    dec = Decoder()
    t0 = time.time()
    last_draw = t0
    count = 0
    try:
        while True:
            data = ser.read(4096)
            for fr in dec.feed(data):
                count += 1
                if csv_file:
                    csv_file.write(",".join(str(fr[k]) for k in FIELDS) + "\n")
                if hist is not None:
                    for k in hist:
                        hist[k].append(fr[k])
                if not args.quiet and count % 200 == 0:
                    print(f"tick={fr['tick']} mode={MODES.get(fr['mode'], fr['mode'])} fault={fr['fault']} "
                          f"iq={fr['i_q']:+.3f}/{fr['i_q_ref']:+.3f} id={fr['i_d']:+.3f} "
                          f"theta_m={fr['theta_m']:.3f} w={fr['omega_m']:+.2f} vq={fr['v_q']:+.2f}")
            if hist is not None and time.time() - last_draw > 0.1 and hist["tick"]:
                x = list(hist["tick"])
                for k, ln in lines.items():
                    ln.set_data(x, list(hist[k]))
                for a in ax:
                    a.relim()
                    a.autoscale_view()
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                last_draw = time.time()
            if args.seconds and time.time() - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file:
            csv_file.close()
        ser.close()
    print(f"{count} frames in {time.time() - t0:.1f} s", file=sys.stderr)


if __name__ == "__main__":
    main()
