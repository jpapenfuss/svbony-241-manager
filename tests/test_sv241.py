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
  DebugLogTests    --debug JSONL trace: record fields, frames/acks/verifies, TUI keys, routing
  DebugImpliesUsbTests  --debug/--debug-text turn on USB watching (silent watcher for one-shots,
                   events for hold), --no-usb, failing USB backend, --debug-text format and
                   fallback, same-file rejection
  UsbLinuxTests    sysfs snapshot on a fake tree: parents, tty ports, peer link, hub finding
  UsbWindowsMacTests  win_build() on fake CfgMgr32 nodes, parse_ioreg() on sample ioreg text
  UsbWatcherTests  diffing, hub scope, mark timing, whole-box unplug, background thread
  UsbCommandTests  cycle (restore, skip-off, rc 3) / on-off windows / usb list+watch / time
                   validation against the simulator, with a fake
                   host USB view in which a camera hangs off the SV241's USB-A output
Hardware-only behaviour (real DTR/RTS reset, boot banner, sensor timing) is not covered, nor
are the real OS USB calls (CfgMgr32 via ctypes, ioreg, a live sysfs).
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
        for v in ("inf", "-1e400", "nan", "1e308", "raw:9999"):
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

    def test_selected_row_keeps_status_text(self):
        t, _ = new_tui()
        select(t, "ch", "pwm1")
        row = [ln for ln in t.lines(130, 48) if "PWM1" in ln and ">" in ln][0]
        self.assertIn("cycles at cutoff edge", row)
        for w in (20, 40, 60):
            self.assertLessEqual(max(sv241.visible_len(ln) for ln in t.lines(w, 48)), w)

    def test_prompt_survives_tiny_terminal(self):
        t, _ = new_tui()
        select(t, "ch", "pwm1")
        t.handle("e")
        self.assertIn("Enter to send", t.lines(80, 1)[-1])
        self.assertEqual(t.lines(80, 0), [])

    def test_pwm1_refusal_keeps_target_when_cutoff_predicted(self):
        t, sim = new_tui()
        select(t, "ch", "pwm1")
        t.handle("h")
        sim.cutoff, sim.saved_pwm1, sim.ch[8], sim.lens_t = True, 253, 0, 30.0
        self.assertFalse(t.holder.in_cutoff)                    # poll hasn't noticed yet
        for k in ["e", *"70", "enter"]:
            t.handle(k)
        self.assertIn("REFUSED", t.msg)
        self.assertEqual(t.holder.targets["pwm1"], sv241.parse_level("pwm1", "70"))

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

    def test_scan_error_not_double_wrapped(self):
        t, sim = new_tui()
        t.hub.open()
        type(sim).in_waiting = property(lambda s: (_ for _ in ()).throw(OSError("gone")))
        try:
            with self.assertRaises(sv241.CommError) as cm:
                t.hub.state()
            self.assertEqual(str(cm.exception).count("serial I/O failed"), 1)
        finally:
            type(sim).in_waiting = 0

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

    def test_raw_bad_bytes_exit_2_without_opening(self):
        from contextlib import redirect_stderr
        for args in (["zz"], ["08", "1ff"], ["01", "08", "10", "00"], ["1_0"], ["+8"], [" 8"],
                     ["-1"], ["0x"], [""]):
            FakeHub.sim = None
            with redirect_stderr(io.StringIO()) as e:
                self.assertEqual(sv241.main(["raw", *args], hub_factory=FakeHub), 2, args)
            self.assertIn("raw", e.getvalue())
            self.assertIsNone(FakeHub.sim, "port must not be opened")

    def test_raw_good_bytes(self):
        FakeHub.sim = FakeSerial()
        with redirect_stdout(io.StringIO()) as o:
            self.assertEqual(sv241.main(["raw", "0x08"], hub_factory=FakeHub), 0)
        self.assertIn("tx 24 06 08 00 00 32", o.getvalue())
        self.assertEqual(sv241.parse_raw_bytes(["1", "ff", "0x0a"]), [1, 255, 10])

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

    def test_unsupported_baud_is_comm_error(self):
        with self.assertRaises(sv241.CommError):
            sv241.PosixSerial(self.dev, baudrate=123457)

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

    def test_ports_listing_tolerates_missing_pid(self):
        orig = sv241.list_serial_ports
        sv241.list_serial_ports = lambda: [("/dev/ttyUSB0", 0x1A86, None, "odd")]
        try:
            with redirect_stdout(io.StringIO()) as out:
                self.assertEqual(sv241.cmd_ports(None), 0)
            self.assertIn("????:????", out.getvalue())
        finally:
            sv241.list_serial_ports = orig

    def test_nothing(self):
        with self.assertRaises(sv241.CommError):
            sv241.pick_port([("/dev/ttyUSB0", 0x0403, 0x6001, "FTDI")])


class DebugLogTests(unittest.TestCase):
    def test_jsonl_trace_of_one_shot(self):
        import json
        import tempfile
        FakeHub.sim = FakeSerial()
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "trace.jsonl")
            with redirect_stdout(io.StringIO()):
                self.assertEqual(sv241.main(["--debug", path, "set", "pwm2", "raw:77"], hub_factory=FakeHub), 0)
            with open(path, encoding="utf-8") as fh:
                recs = [json.loads(ln) for ln in fh]
        self.assertFalse(sv241.DBG.on, "closed after main()")
        evs = [r["ev"] for r in recs]
        self.assertEqual(evs[0], "start")
        self.assertEqual(recs[-1], dict(recs[-1], ev="exit", rc=0))
        for r in recs:
            self.assertIn("ts", r)
            self.assertIsInstance(r["mono"], float)
        x = next(r for r in recs if r["ev"] == "xfer" and r["tx"].startswith("24 06 01"))
        self.assertEqual(x["tx"], sv241.frame(1, 9, 77).hex(" "))
        self.assertEqual(x["got"], 6)
        self.assertIn({"ev": "ack", "target": "pwm2", "result": "ok"},
                      [{k: r.get(k) for k in ("ev", "target", "result")} for r in recs])
        v = next(r for r in recs if r["ev"] == "verify")
        self.assertTrue(v["ok"])

    def test_debug_routed_as_global_and_stderr_refused_for_tui(self):
        from contextlib import redirect_stderr
        f = sv241.insert_default_command
        self.assertEqual(f(["--pwm1", "40", "--debug", "x.jsonl"]), ["--debug", "x.jsonl", "hold", "--pwm1", "40"])
        self.assertEqual(f(["--usb-hub=1-2", "tui"]), ["--usb-hub=1-2", "tui"])
        with redirect_stderr(io.StringIO()) as e:
            self.assertEqual(sv241.main(["--debug", "-", "tui"], hub_factory=FakeHub), 2)
        self.assertIn("garble", e.getvalue())

    def test_tui_keys_and_events_logged(self):
        import json
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "t.jsonl")
            sv241.DBG.open(path)
            try:
                t, sim = new_tui()
                t.handle("x")
                t.handle("y")
            finally:
                sv241.DBG.close()
            with open(path, encoding="utf-8") as fh:
                recs = [json.loads(ln) for ln in fh]
        keys = [r["key"] for r in recs if r["ev"] == "key"]
        self.assertEqual(keys, ["x", "y"])
        self.assertTrue(any(r["ev"] == "event" and "all outputs switched off" in r["msg"] for r in recs))


def make_sysfs(root, devices, peers=()):
    """Fake /sys/bus/usb/devices: devices = {name: (vid, pid, product, [tty names])};
    peers = [(hub iface, port dir, target iface, target port dir)] creating peer symlinks."""
    for name, (vid, pid, prod, ttys) in devices.items():
        p = os.path.join(root, name)
        os.makedirs(p)
        for f, v in (("idVendor", f"{vid:04x}"), ("idProduct", f"{pid:04x}"), ("product", prod)):
            with open(os.path.join(p, f), "w") as fh:
                fh.write(v + "\n")
        for t in ttys:
            os.makedirs(os.path.join(p, f"{name}:1.0", t))
    for hub_if, port, t_if, t_port in peers:
        os.makedirs(os.path.join(root, hub_if, port), exist_ok=True)
        os.makedirs(os.path.join(root, t_if, t_port), exist_ok=True)
        os.symlink(os.path.join(root, t_if, t_port), os.path.join(root, hub_if, port, "peer"))


LINUX_TREE = {
    "usb1": (0x1D6B, 0x0002, "xHCI Host Controller", []),
    "usb2": (0x1D6B, 0x0003, "xHCI Host Controller", []),
    "1-2": (0x2109, 0x2817, "USB2.0 Hub", []),             # SV241 internal hub, USB2 half
    "2-2": (0x2109, 0x0817, "USB3.0 Hub", []),             # ... and its SuperSpeed half
    "1-2.1": (0x1A86, 0x7523, "USB Serial", ["ttyUSB0"]),  # the SV241's CH340
    "1-2.3": (0x03C3, 0x120E, "ZWO ASI120MM", []),
    "2-2.2": (0x03C3, 0x2626, "ZWO ASI2600MC", []),
    "1-4": (0x046D, 0xC52B, "Unifying Receiver", []),
}


@unittest.skipIf(os.name == "nt", "symlinks")
class UsbLinuxTests(unittest.TestCase):
    def snap(self, peers=True, tree=LINUX_TREE):
        import tempfile
        d = tempfile.mkdtemp()
        self.addCleanup(__import__("shutil").rmtree, d)
        make_sysfs(d, tree, [("1-0:1.0", "usb1-port2", "2-0:1.0", "usb2-port2")] if peers else [])
        return sv241.linux_usb_snapshot(d)

    def test_tree_ports_and_peer(self):
        s = self.snap()
        self.assertNotIn("1-2.1:1.0", s)
        self.assertEqual(s["1-2.1"].parent, "1-2")
        self.assertEqual(s["1-2"].parent, "usb1")
        self.assertTrue(s["usb1"].root)
        self.assertEqual(s["1-2.1"].ports, ("/dev/ttyUSB0",))
        self.assertEqual(s["1-2"].peer, "2-2")
        self.assertEqual(sv241._linux_dev_at_port("2-2-port3"), "2-2.3")

    def test_hub_found_from_ch340_with_companion(self):
        s = self.snap()
        hubs, bridge, note = sv241.find_sv241_hubs(s, "/dev/ttyUSB0")
        self.assertEqual((hubs, bridge, note), (["1-2", "2-2"], "1-2.1", None))
        self.assertEqual(sv241.usb_subtree(s, hubs), {"1-2", "2-2", "1-2.1", "1-2.3", "2-2.2"})

    def test_companion_heuristic_without_peer_link(self):
        hubs, _, _ = sv241.find_sv241_hubs(self.snap(peers=False))
        self.assertEqual(hubs, ["1-2", "2-2"])

    def test_ch340_on_root_port_is_not_a_hub(self):
        tree = {"usb1": LINUX_TREE["usb1"], "1-3": (0x1A86, 0x7523, "USB Serial", ["ttyUSB0"])}
        hubs, bridge, note = sv241.find_sv241_hubs(self.snap(tree=tree))
        self.assertEqual((hubs, bridge), ([], "1-3"))
        self.assertIn("--usb-hub", note)

    def test_hub_override(self):
        s = self.snap()
        self.assertEqual(sv241.find_sv241_hubs(s, spec="2109:2817")[0], ["1-2"])
        hubs, _, note = sv241.find_sv241_hubs(s, spec="1-4,9-9")
        self.assertEqual(hubs, ["1-4"])
        self.assertIn("9-9", note)


class UsbWindowsMacTests(unittest.TestCase):
    def test_win_build_folds_interfaces_and_groups_container(self):
        c = "{aaaa0000-1111-2222-3333-444455556666}"
        nodes = [
            ("USB\\ROOT_HUB30\\4&1&0", "PCI\\VEN_8086", "", "USB Root Hub (USB 3.0)", ""),
            ("USB\\VID_2109&PID_2817\\5&2&0&2", "USB\\ROOT_HUB30\\4&1&0", "", "Generic USB Hub", c),
            ("USB\\VID_2109&PID_0817\\5&2&0&14", "USB\\ROOT_HUB30\\4&1&0", "", "@usb.inf,%hub%;Generic SuperSpeed USB Hub", c),
            ("USB\\VID_1A86&PID_7523\\6&3&0&1", "USB\\VID_2109&PID_2817\\5&2&0&2", "USB-SERIAL CH340 (COM6)", "", c),
            ("USB\\VID_0403&PID_6015\\DN05ABCD", "USB\\VID_2109&PID_2817\\5&2&0&2", "", "USB Serial Converter", ""),
            ("FTDIBUS\\VID_0403+PID_6015+DN05ABCDA\\0000", "USB\\VID_0403&PID_6015\\DN05ABCD", "USB Serial Port (COM9)", "", ""),
            ("USB\\VID_03C3&PID_2626&MI_00\\7&1", "USB\\VID_03C3&PID_2626\\0123", "", "ASI interface", ""),
            ("USB\\VID_03C3&PID_2626\\0123", "USB\\VID_2109&PID_0817\\5&2&0&14", "ZWO ASI2600MC", "", ""),
        ]
        s = sv241.win_build(nodes)
        self.assertNotIn("USB\\VID_03C3&PID_2626&MI_00\\7&1", s)
        self.assertNotIn("FTDIBUS\\VID_0403+PID_6015+DN05ABCDA\\0000", s)
        ftdi = s["USB\\VID_0403&PID_6015\\DN05ABCD"]
        self.assertEqual((ftdi.ports, ftdi.serial), (("COM9",), "DN05ABCD"))
        self.assertEqual(s["USB\\VID_2109&PID_0817\\5&2&0&14"].name, "Generic SuperSpeed USB Hub")
        self.assertTrue(s["USB\\ROOT_HUB30\\4&1&0"].root)
        hubs, bridge, note = sv241.find_sv241_hubs(s, "COM6")
        self.assertEqual(hubs, ["USB\\VID_2109&PID_2817\\5&2&0&2", "USB\\VID_2109&PID_0817\\5&2&0&14"])
        self.assertEqual(bridge, "USB\\VID_1A86&PID_7523\\6&3&0&1")
        self.assertIn("USB\\VID_03C3&PID_2626\\0123", sv241.usb_subtree(s, hubs))

    def test_parse_ioreg(self):
        text = "\n".join([
            "+-o Root  <class IORegistryEntry, id 0x100000100, retain 26>",
            "  +-o AppleT8103USBXHCI@00000000  <class AppleT8103USBXHCI, id 0x1000002a1>",
            "  | +-o USB2.0 Hub@00100000  <class IOUSBHostDevice, id 0x100000a01, registered>",
            "  | | {",
            '  | |   "idProduct" = 10263',
            '  | |   "idVendor" = 8457',
            '  | |   "USB Product Name" = "USB2.0 Hub"',
            '  | |   "locationID" = 1048576',
            "  | | }",
            "  | | ",
            "  | | +-o USB Serial@00110000  <class IOUSBHostDevice, id 0x100000a11, registered>",
            "  | |     {",
            '  | |       "idProduct" = 29987',
            '  | |       "idVendor" = 6790',
            '  | |       "locationID" = 1114112',
            "  | |     }",
            "  +-o AppleT8103USBXHCI@01000000  <class AppleT8103USBXHCI, id 0x1000002b1>",
            "    +-o Keyboard@01100000  <class IOUSBHostDevice, id 0x100000b01>",
            "        {",
            '          "idVendor" = 1452',
            '          "idProduct" = 641',
            '          "locationID" = 17825792',
            "        }",
        ])
        s = sv241.parse_ioreg(text)
        self.assertEqual(sorted(s), ["0x00100000", "0x00110000", "0x01100000"])
        ch = s["0x00110000"]
        self.assertEqual((ch.vid, ch.pid, ch.parent, ch.name), (0x1A86, 0x7523, "0x00100000", "USB Serial"))
        self.assertIsNone(s["0x00100000"].parent)
        self.assertIsNone(s["0x01100000"].parent)
        self.assertEqual(sv241.find_sv241_hubs(s)[0], ["0x00100000"])


def dev(key, parent, vid, pid, name, ports=(), root=False):
    return sv241.UsbDev(key, parent, vid, pid, name, "", ports, root)


ROOT = dev("usb1", None, 0x1D6B, 2, "root", root=True)
HUB = dev("1-2", "usb1", 0x2109, 0x2817, "USB2.0 Hub")
CH = dev("1-2.1", "1-2", 0x1A86, 0x7523, "USB Serial", ("SIM",))
CAM = dev("1-2.3", "1-2", 0x03C3, 0x120E, "ZWO ASI120MM")
MOUSE = dev("1-4", "usb1", 0x046D, 0xC52B, "mouse")


def sim_snapshot():
    """Host USB view that follows the simulator: the camera is on the SV241's USB-A output."""
    s = {d.key: d for d in (ROOT, HUB, CH, MOUSE)}
    if FakeHub.sim is not None and FakeHub.sim.outs[6]:
        s[CAM.key] = CAM
    return s


class UsbWatcherTests(unittest.TestCase):
    def test_hub_scope_changes_and_mark_timing(self):
        FakeHub.sim = FakeSerial()
        seq = [sim_snapshot()]
        lines = []
        w = sv241.UsbWatcher("hub", sink=lines.append, snapshot=lambda: seq[-1])
        self.assertEqual(w.poll(), [])                      # baseline
        self.assertEqual(w.hubs, ["1-2"])
        self.assertEqual(w.count(), 3)                      # hub, CH340, camera; not the mouse
        w.mark("USB-A off")
        FakeHub.sim.outs[6] = 0
        seq.append(dict(sim_snapshot()))
        seq[-1].pop("1-4")                                  # out of scope: not reported
        ch = w.poll()
        self.assertEqual([(k, d.key) for k, d in ch], [("-", "1-2.3")])
        self.assertRegex(lines[-1], r"DISCONNECTED 03C3:120E ZWO ASI120MM \[1-2\.3\]  \+\d\.\d\d s after USB-A off")

    def test_whole_box_unplugged_and_back(self):
        full = sim_snapshot()
        seq = [full]
        lines = []
        w = sv241.UsbWatcher("hub", sink=lines.append, snapshot=lambda: seq[-1])
        w.poll()
        seq.append({"usb1": ROOT, "1-4": MOUSE})
        gone = w.poll()
        self.assertEqual({d.key for _, d in gone}, set(full) - {"usb1", "1-4"})
        self.assertTrue(any("gone from the bus" in x for x in lines))
        seq.append(full)
        back = w.poll()
        self.assertEqual({d.key for k, d in back if k == "+"}, set(full) - {"usb1", "1-4"})

    def test_unavailable_said_once(self):
        lines = []

        def boom():
            raise sv241.UsbUnavailable("nope")
        w = sv241.UsbWatcher(sink=lines.append, snapshot=boom)
        w.poll()
        w.poll()
        self.assertFalse(w.available)
        self.assertEqual(lines, ["USB monitoring unavailable: nope"])

    def test_background_thread_feeds_holder_events(self):
        FakeHub.sim = FakeSerial()
        a = sv241.build_parser().parse_args(["hold", "--usb-watch", "--usb-interval", "0.1"])
        h = sv241.Holder(a, FakeHub(None, 0.4))
        orig = sv241.usb_snapshot
        sv241.usb_snapshot = sim_snapshot
        try:
            h.start_usb()
            FakeHub.sim.outs[6] = 0
            import time as _t
            for _ in range(50):
                if any("DISCONNECTED" in e for e in h.events):
                    break
                _t.sleep(0.05)
        finally:
            h.stop_usb()
            sv241.usb_snapshot = orig
        self.assertTrue(any("DISCONNECTED" in e and "ASI120MM" in e for e in h.events), list(h.events))


class DebugImpliesUsbTests(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.orig = sv241.usb_snapshot
        sv241.usb_snapshot = sim_snapshot
        FakeHub.sim = FakeSerial()
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)

    def tearDown(self):
        sv241.usb_snapshot = self.orig

    def read(self, name):
        import json
        with open(os.path.join(self.dir.name, name), encoding="utf-8") as fh:
            return [json.loads(ln) for ln in fh] if name.endswith(".jsonl") else fh.read()

    def test_one_shot_debug_runs_silent_usb_watcher(self):
        j = os.path.join(self.dir.name, "t.jsonl")
        with redirect_stdout(io.StringIO()) as o:
            self.assertEqual(sv241.main(["--debug", j, "off", "dc2"], hub_factory=FakeHub), 0)
        self.assertNotIn("USB", o.getvalue(), "background watcher prints nothing")
        evs = [r["ev"] for r in self.read("t.jsonl")]
        self.assertIn("usb_baseline", evs)
        self.assertIn("usb_hub", evs)
        with redirect_stdout(io.StringIO()):
            sv241.main(["--debug", j, "--no-usb", "status"], hub_factory=FakeHub)
        self.assertEqual([r["ev"] for r in self.read("t.jsonl")].count("usb_baseline"), 1, "--no-usb")

    def test_failing_usb_backend_never_breaks_a_command(self):
        def broken():
            raise OSError("backend exploded")
        sv241.usb_snapshot = broken
        j = os.path.join(self.dir.name, "f.jsonl")
        with redirect_stdout(io.StringIO()):
            self.assertEqual(sv241.main(["--debug", j, "status"], hub_factory=FakeHub), 0)
        errs = [r for r in self.read("f.jsonl") if r["ev"] == "usb_error"]
        self.assertEqual(len(errs), 1)
        self.assertIn("backend exploded", errs[0]["error"])

    def test_same_file_for_both_formats_rejected(self):
        from contextlib import redirect_stderr
        j = os.path.join(self.dir.name, "same")
        with redirect_stderr(io.StringIO()) as e:
            self.assertEqual(sv241.main(["--debug", j, "--debug-text", j, "status"], hub_factory=FakeHub), 2)
        self.assertIn("different files", e.getvalue())

    def test_text_formatter_falls_back_to_generic(self):
        rec = {"ts": "2026-11-22T06:00:01.234+00:00", "ev": "xfer", "tx": "24"}   # no rx/got/ms
        self.assertRaises(KeyError, sv241.fmt_debug_text, rec)
        self.assertEqual(sv241.fmt_debug_text(rec, generic=True), "06:00:01.234  xfer         tx=24")

    def test_hold_debug_implies_usb_watch(self):
        a = sv241.build_parser().parse_args(["--debug", "x", "hold"])
        self.assertTrue(sv241.usb_watch_wanted(a))
        a = sv241.build_parser().parse_args(sv241.insert_default_command(["--pwm1", "40", "--no-usb", "--debug-text", "x"]))
        self.assertFalse(sv241.usb_watch_wanted(a))
        self.assertFalse(sv241.usb_watch_wanted(sv241.build_parser().parse_args(["hold"])))

    def test_text_and_jsonl_side_by_side(self):
        j, t = (os.path.join(self.dir.name, n) for n in ("a.jsonl", "a.log"))
        with redirect_stdout(io.StringIO()):
            self.assertEqual(sv241.main(["--debug", j, "--debug-text", t, "--no-usb", "status"], hub_factory=FakeHub), 0)
        recs, text = self.read("a.jsonl"), self.read("a.log")
        self.assertEqual(len(recs), len(text.splitlines()))
        self.assertRegex(text, r"\d\d:\d\d:\d\d\.\d{3}  xfer         tx 24 06 08 00 00 32  ->  rx 24 0e 08 .* 14/14 B")
        self.assertIn("state        outputs DC1=on", text)
        self.assertRegex(text.splitlines()[-1], r"exit +rc=0$")

    def test_text_format_of_usb_and_short_reply(self):
        f = sv241.fmt_debug_text
        rec = {"ts": "2026-11-22T06:00:01.234+00:00", "mono": 1.0, "ev": "usb_remove",
               "dev": sv241.usb_record(CAM), "below_hub": True, "mark": "USB-A off", "since_mark_s": 0.42}
        self.assertEqual(f(rec), "06:00:01.234  usb_remove   - 03c3:120e ZWO ASI120MM [1-2.3] behind SV241 hub"
                                 "  +0.42 s after USB-A off")
        rec = {"ts": "2026-11-22T06:00:01.234+00:00", "ev": "xfer", "tx": "24 06 08 00 00 32", "rx": "",
               "expect": 14, "got": 0, "ms": 400.2}
        self.assertTrue(f(rec).endswith("rx (nothing)  0/14 B  400.2 ms  TIMEOUT/SHORT"))


class UsbCommandTests(unittest.TestCase):
    def setUp(self):
        self.orig = sv241.usb_snapshot
        sv241.usb_snapshot = sim_snapshot
        FakeHub.sim = FakeSerial()

    def tearDown(self):
        sv241.usb_snapshot = self.orig

    def test_cycle_reports_gone_and_back(self):
        with redirect_stdout(io.StringIO()) as o:
            rc = sv241.main(["cycle", "usba", "--off-time", "0.3", "--watch", "0.3"], hub_factory=FakeHub)
        out = o.getvalue()
        self.assertEqual(rc, 0, out)
        self.assertEqual(FakeHub.sim.outs[6], 0xFF, "restored")
        self.assertRegex(out, r"gone \+\d\.\d\d s after off, back \+\d\.\d\d s after on: 03C3:120E")
        self.assertIn("(behind SV241 hub)", out)

    def test_cycle_restores_others_when_one_is_refused(self):
        class Stubborn(FakeHub):
            def set_output(self, idx, on):
                if idx == 1 and on:
                    raise sv241.Refused("DC2 still off (test)")
                return super().set_output(idx, on)
        from contextlib import redirect_stderr
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()) as e:
            rc = sv241.main(["cycle", "dc1", "dc2", "dc3", "usbc", "--off-time", "0", "--watch", "0"],
                            hub_factory=Stubborn)
        self.assertEqual(rc, 2)
        outs = FakeHub.sim.outs
        self.assertEqual([outs[0], outs[1], outs[2], outs[5]], [0xFF, 0, 0xFF, 0xFF])
        self.assertIn("STILL OFF after cycle: DC2", e.getvalue())

    def test_cycle_skips_outputs_that_are_off(self):
        FakeHub.sim.outs[0] = 0
        with redirect_stdout(io.StringIO()) as o:
            self.assertEqual(sv241.main(["cycle", "dc1", "--off-time", "0", "--watch", "0"], hub_factory=FakeHub), 0)
        self.assertIn("DC1 is off; not cycled", o.getvalue())
        self.assertEqual(FakeHub.sim.outs[0], 0)

    def test_cycle_missing_device_rc3(self):
        def stuck():
            s = sim_snapshot()
            if FakeHub.sim.log and any(f[2] == 0x01 and f[3] == 6 and f[4] == 0 for f in FakeHub.sim.log):
                s.pop(CAM.key, None)                        # never comes back once switched off
            return s
        sv241.usb_snapshot = stuck
        with redirect_stdout(io.StringIO()) as o:
            rc = sv241.main(["cycle", "usba", "--off-time", "0.2", "--watch", "0.2"], hub_factory=FakeHub)
        self.assertEqual(rc, 3)
        self.assertIn("NOT back within", o.getvalue())

    def test_off_usb_output_watches_window(self):
        with redirect_stdout(io.StringIO()) as o:
            self.assertEqual(sv241.main(["off", "usba", "--usb-window", "0.3"], hub_factory=FakeHub), 0)
        self.assertRegex(o.getvalue(), r"DISCONNECTED .*ASI120MM.*after USB-A off")
        with redirect_stdout(io.StringIO()) as o:
            self.assertEqual(sv241.main(["on", "dc2"], hub_factory=FakeHub), 0)
        self.assertNotIn("watching USB", o.getvalue(), "DC outputs: no window by default")

    def test_bad_times_rejected_before_opening(self):
        from contextlib import redirect_stderr
        FakeHub.sim = None
        with redirect_stderr(io.StringIO()):
            self.assertEqual(sv241.main(["cycle", "usba", "--off-time", "-1"], hub_factory=FakeHub), 2)
            self.assertEqual(sv241.main(["cycle", "bogus"], hub_factory=FakeHub), 2)
        self.assertIsNone(FakeHub.sim)

    def test_usb_list_and_watch(self):
        with redirect_stdout(io.StringIO()) as o:
            self.assertEqual(sv241.main(["--port", "SIM", "usb"], hub_factory=FakeHub), 0)
        out = o.getvalue()
        self.assertIn("USB2.0 Hub [1-2]   <- SV241 hub", out)
        self.assertIn("USB Serial (SIM) [1-2.1]   <- SV241 serial bridge", out)
        self.assertIn("ZWO ASI120MM [1-2.3]   (behind SV241)", out)
        a = sv241.build_parser().parse_args(["usb", "--watch", "--interval", "0.1"])
        a.max_polls = 1
        calls = []

        def flip():                       # camera unplugs right after the baseline
            calls.append(1)
            if len(calls) == 2:
                FakeHub.sim.outs[6] = 0
            return sim_snapshot()
        sv241.usb_snapshot = flip
        with redirect_stdout(io.StringIO()) as o:
            self.assertEqual(sv241.cmd_usb(a), 0)
        self.assertIn("watching 3 devices behind the SV241 hub", o.getvalue())
        self.assertIn("DISCONNECTED 03C3:120E", o.getvalue())


if __name__ == "__main__":
    unittest.main()
