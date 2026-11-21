"""
Simulator-backed tests for sv241.py: python -m unittest discover -s tests

No hardware or pyserial needed. Most tests drive sv241 against tests/fake_hub.py (FakeSerial /
FakeHub). Coverage:
  TuiTests         key handling, confirmations, debounced writes, holds, PWM1 cutoff refusal,
                   rendering limits, comm errors (Tui driven directly; no real terminal)
  KeyParseTests    POSIX escape-sequence parsing over a pipe
  CliTests         default-command routing (old hold.py command lines), one-shot commands
  PosixSerialTests the stdlib termios backend end to end over a pty pair (POSIX only)
  PortPickTests    auto-detect rules: CH340 preference, cu.* over tty.*, guarded guesses
Hardware-only behaviour (real DTR/RTS reset, boot banner, sensor timing) is not covered.
"""
import io
import os
import sys
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_hub import FakeHub, FakeSerial, sv241  # noqa: E402


def new_tui(*argv):
    """Fresh simulator + Tui parsed from `tui *argv`, output to a StringIO, one poll done."""
    FakeHub.sim = FakeSerial()
    a = sv241.build_parser().parse_args(["tui", *argv])
    hub = FakeHub(None, 0.4)
    t = sv241.Tui(a, hub, out=io.StringIO())
    t.poll()
    return t, FakeHub.sim


def select(t, kind, which):
    """Move the TUI selection to row (kind, which), e.g. ("ch", "pwm2")."""
    t.sel = sv241.TUI_ROWS.index((kind, which))


class TuiTests(unittest.TestCase):
    def test_output_on_and_confirmed_off(self):
        t, sim = new_tui()
        select(t, "out", 1)
        t.handle(" ")
        self.assertEqual(sim.outs[1], 0xFF, "off needs confirmation")
        t.handle("n")
        self.assertEqual(sim.outs[1], 0xFF)
        t.handle(" ")
        t.handle("y")
        self.assertEqual(sim.outs[1], 0x00)
        t.poll()
        t.handle("enter")                      # off -> on needs no confirmation
        self.assertEqual(sim.outs[1], 0xFF)

    def test_all_off_confirm_and_all_on(self):
        t, sim = new_tui()
        t.handle("x")
        t.handle("y")
        self.assertEqual(sim.outs, [0] * 7)
        t.handle("a")
        self.assertEqual(sim.outs, [0xFF] * 7)

    def test_steps_are_debounced_into_one_write(self):
        t, sim = new_tui()
        select(t, "ch", "pwm2")
        for _ in range(3):
            t.handle("left")
        t.handle(",")
        n0 = len(sim.log)
        t.flush_pending()                      # too soon: nothing sent
        self.assertEqual(len(sim.log), n0)
        t.pending["pwm2"][1] -= sv241.DEBOUNCE_S
        t.flush_pending()
        want = 253 - 3 * sv241.COARSE["pwm2"] - 1
        self.assertEqual(sim.ch[9], want)
        sets = [f for f in sim.log[n0:] if f[2] == 0x01]
        self.assertEqual(len(sets), 1)

    def test_edit_value_and_bad_value(self):
        t, sim = new_tui()
        select(t, "ch", "adj")
        for k in ["e", *"raw:120", "enter"]:
            t.handle(k)
        self.assertEqual(sim.ch[7], 120)
        for k in ["e", *"999", "enter"]:
            t.handle(k)
        self.assertEqual(sim.ch[7], 120)
        self.assertIn("bad value", t.msg)
        for k in ["e", *"12", "backspace", "esc"]:
            t.handle(k)
        self.assertEqual(sim.ch[7], 120)

    def test_non_finite_value_is_rejected(self):
        t, sim = new_tui()
        select(t, "ch", "pwm2")
        for v in ("inf", "-1e400", "nan"):
            for k in ["e", *v, "enter"]:
                t.handle(k)
            self.assertIn("bad value", t.msg)
        self.assertEqual(sim.ch[9], 253)

    def test_refused_write_restores_hold_target(self):
        t, sim = new_tui()
        select(t, "ch", "pwm2")
        t.handle("h")
        orig = sim.write

        def clamp(b):                           # hub "accepts" but stores something else
            orig(b)
            if b[2] == 0x01 and b[3] == 9:
                sim.ch[9] = 50
        sim.write = clamp
        for k in ["e", *"raw:120", "enter"]:
            t.handle(k)
        self.assertIn("REFUSED", t.msg)
        self.assertEqual(t.holder.targets["pwm2"], 253)

    def test_prompt_visible_on_small_terminal(self):
        t, _ = new_tui()
        select(t, "ch", "pwm1")
        t.handle("e")
        ls = t.lines(80, 24)
        self.assertEqual(len(ls), 24)
        self.assertTrue(any("Enter to send" in ln for ln in ls))

    def test_pwm_step_capped_at_100(self):
        t, sim = new_tui()
        select(t, "ch", "pwm2")
        t.handle("right")
        self.assertEqual(t.pending["pwm2"][0], sv241.RAW_FULL)

    def test_hold_repushes_and_follows_tui_changes(self):
        t, sim = new_tui()
        select(t, "ch", "pwm2")
        t.handle("h")
        self.assertEqual(t.holder.targets["pwm2"], 253)
        for k in ["e", *"40", "enter"]:
            t.handle(k)
        self.assertEqual(t.holder.targets["pwm2"], sv241.parse_level("pwm2", "40"))
        sim.ch[9] = 7                          # someone else changes it
        t.poll()
        self.assertEqual(sim.ch[9], t.holder.targets["pwm2"])
        t.handle("h")
        self.assertNotIn("pwm2", t.holder.targets)

    def test_pwm1_refused_during_cutoff(self):
        t, sim = new_tui()
        sim.cutoff, sim.saved_pwm1, sim.ch[8] = True, 253, 0
        sim.lens_t = 30.0
        select(t, "ch", "pwm1")
        for k in ["e", *"50", "enter"]:
            t.handle(k)
        self.assertIn("REFUSED", t.msg)
        self.assertEqual(sim.ch[8], 0)

    def test_quit_flushes_pending(self):
        t, sim = new_tui()
        select(t, "ch", "pwm2")
        t.handle("left")

        class K:
            def __enter__(s): return s
            def __exit__(s, *e): pass
            def read(s, timeout): return ["q"]
        t.keys = K()
        t.run()
        self.assertEqual(sim.ch[9], 253 - sv241.COARSE["pwm2"])
        self.assertTrue(t.out.getvalue().endswith("\033[?25h\033[?1049l"))

    def test_render_fits_terminal(self):
        t, _ = new_tui()
        for w, h in ((80, 24), (130, 48), (40, 10)):
            ls = t.lines(w, h)
            self.assertLessEqual(len(ls), h)
            for ln in ls:
                self.assertLessEqual(sv241.visible_len(ln), w, ln)

    def test_comm_error_is_reported_not_fatal(self):
        t, sim = new_tui()
        sim.write = lambda b: None             # hub stops answering
        t.hub.timeout = 0.01
        t.poll()
        self.assertIsNotNone(t.err)
        self.assertEqual(t.holder.counts["comm"], 1)
        t.lines(100, 40)


class KeyParseTests(unittest.TestCase):
    def test_posix_sequences(self):
        r, w = os.pipe()
        k = sv241.Keys()
        k.fd = r
        os.write(w, b"\x1b[A\x1b[Bq\r\x1b\x7f")
        self.assertEqual(k._read_posix(0.5), ["up", "down", "q", "enter", "esc", "backspace"])
        self.assertEqual(k._read_posix(0.01), [])
        os.close(r)
        os.close(w)


class CliTests(unittest.TestCase):
    def test_default_command_routing(self):
        f = sv241.insert_default_command
        self.assertEqual(f(["--pwm1", "40", "--port", "COM6"]), ["--port", "COM6", "hold", "--pwm1", "40"])
        self.assertEqual(f(["--port", "COM6", "tui"]), ["--port", "COM6", "tui"])

    def test_one_shot_against_sim(self):
        FakeHub.sim = FakeSerial()
        with redirect_stdout(io.StringIO()):
            self.assertEqual(sv241.main(["off", "dc3"], hub_factory=FakeHub), 0)
            self.assertEqual(sv241.main(["set", "pwm2", "raw:77"], hub_factory=FakeHub), 0)
        self.assertEqual(FakeHub.sim.outs[2], 0)
        self.assertEqual(FakeHub.sim.ch[9], 77)

    def test_tui_refuses_non_tty(self):
        from contextlib import redirect_stderr
        with redirect_stderr(io.StringIO()) as e:
            rc = sv241.main(["tui"], hub_factory=FakeHub)
        self.assertEqual(rc, 2)
        self.assertIn("interactive console", e.getvalue())


@unittest.skipIf(os.name == "nt", "POSIX termios backend")
class PosixSerialTests(unittest.TestCase):
    """Runs the stdlib termios backend over a pty pair, with the simulator answering on the
    master side: the full Hub path, minus a real USB adapter."""

    def setUp(self):
        import pty
        import threading
        self.master, self.slave = pty.openpty()      # keep slave open: else master reads hit EIO
        self.dev = os.ttyname(self.slave)
        self.sim = FakeSerial()
        self.stop = False

        def serve():
            import select
            buf = b""
            while not self.stop:
                r, _, _ = select.select([self.master], [], [], 0.05)
                if not r:
                    continue
                try:
                    buf += os.read(self.master, 256)
                except OSError:
                    return
                while len(buf) >= 6:
                    self.sim.write(buf[:6])
                    buf = buf[6:]
                    if self.sim.rx:
                        os.write(self.master, bytes(self.sim.rx))
                        self.sim.rx.clear()
        self.t = threading.Thread(target=serve, daemon=True)
        self.t.start()

    def tearDown(self):
        self.stop = True
        self.t.join(1)
        os.close(self.master)
        os.close(self.slave)

    def test_hub_over_termios(self):
        orig = sv241.have_pyserial
        sv241.have_pyserial = lambda: False
        try:
            hub = sv241.Hub(self.dev, 0.4)
            with hub:
                self.assertIsInstance(hub.ser, sv241.PosixSerial)
                outs, dev = hub.state()
                self.assertEqual(outs, [True] * 7)
                hub.set_channel("pwm2", 100)
                hub.set_output(4, False)
                s = hub.sensors()
                self.assertAlmostEqual(s["input_v"], 12.4, places=2)
                with self.assertRaises(sv241.CommError):     # flock: second opener refused
                    sv241.PosixSerial(self.dev)
            self.assertEqual(self.sim.ch[9], 100)
            self.assertEqual(self.sim.outs[4], 0)
        finally:
            sv241.have_pyserial = orig


class PortPickTests(unittest.TestCase):
    def test_prefers_ch340_and_cu(self):
        ports = [("/dev/ttyUSB0", 0x0403, 0x6001, "FTDI"), ("/dev/tty.wchusbserial1", *sv241.CH340, ""),
                 ("/dev/cu.wchusbserial1", *sv241.CH340, ""), ("/dev/ttyUSB1", *sv241.CH340, "")]
        self.assertEqual(sv241.pick_port(ports)[0], "/dev/cu.wchusbserial1")

    def test_lone_unidentified_port_is_a_guess(self):
        dev, note = sv241.pick_port([("/dev/cu.usbserial-1410", None, None, "USB serial"),
                                     ("/dev/tty.usbserial-1410", None, None, "USB serial")])
        self.assertEqual(dev, "/dev/cu.usbserial-1410")
        self.assertIn("trying", note)

    def test_non_usb_ports_are_never_guessed(self):
        for ports in ([("COM1", None, None, "Communications Port")],
                      [("/dev/cu.Bluetooth-Incoming-Port", None, None, "")],
                      [("/dev/ttyS0", None, None, "")]):
            with self.assertRaises(sv241.CommError):
                sv241.pick_port(ports)

    @unittest.skipIf(os.name == "nt", "POSIX termios backend")
    def test_non_tty_is_comm_error(self):
        with self.assertRaises(sv241.CommError):
            sv241.PosixSerial("/dev/null")

    def test_nothing(self):
        with self.assertRaises(sv241.CommError):
            sv241.pick_port([("/dev/ttyUSB0", 0x0403, 0x6001, "FTDI")])


if __name__ == "__main__":
    unittest.main()
