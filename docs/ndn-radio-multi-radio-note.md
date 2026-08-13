# Note for the ndn-workspace session — device selection when a host has multiple identical radios

**From:** miniMUAS v2 named-data-radio pilot (aarch64 fleet bring-up)
**Re:** `ndn-radio-drivers` — `LibUsbRtl88xxBackend` (RTL8822E / PID `0bda:a81a`) claims the *first* matching USB device with no way to pick among several identical dongles. On our two-radio nodes that forces the radio face onto the radio currently carrying the kernel Wi-Fi link.

## Hardware context (minidronesys fleet)
Each node (Odroid-class aarch64) has **two RTL8812EU dongles**, both enumerating as
`0bda:a81a` ("802.11ac NIC"), both bound to the kernel `rtl88x2eu` driver:

```
mesh0  rtl88x2eu  192.168.1.12/24  ch149   USB 1-1.1 (Dev 003)   <- ACTIVE fleet Wi-Fi mesh (v2 NDN link + time sync)
wlan0  rtl88x2eu  DOWN, no IP               USB 1-1.4 (Dev 005)   <- spare, free for a named-data-radio face
end0   meson8b-dwmac  141.225.x  (wired)                          <- SSH / management (unaffected by radio claims)
```

Intent for the A/B pilot: dedicate the **spare** radio (`wlan0`, `1-1.4`) to the
`[[face]] kind="radio"` medium face while `mesh0` keeps running the Wi-Fi cell, so a
single node can host both the `v2/wifi` and `v2/radio` comparison cells and, more
importantly, so bringing up the radio face never disturbs the live fleet mesh.

## The limitation
`open_named_radio(pid=0xa81a, …)` routes to the **RTL8822E branch**, which calls
`LibUsbRtl88xxBackend::open_monitor_pid` → `open_pid`:

```rust
// ndn-radio-drivers/src/libusb_rtl88xx.rs
pub fn open_pid(want_pid: u16) -> Result<Self, FaceError> {
    let context = Context::new()?;
    for device in context.devices()?.iter() {
        let desc = device.device_descriptor()?;
        if desc.vendor_id() == REALTEK_VID && desc.product_id() == want_pid {
            return Self::claim(device);   // <-- FIRST match, unconditionally
        }
    }
    Err(… "no Realtek 0bda:{want_pid:04x} found")
}
```

There is **no index / bus / address / MAC selector** on this path. With two `0xa81a`
devices it always claims the first-enumerated one (Dev 003 = `mesh0`, our active link).
Claiming also detaches the kernel `rtl88x2eu` driver from that device, so the node
leaves the 192.168.1.x mesh; release does not cleanly rebind the kernel driver.

By contrast, the **RTL8812AU branch** (chip 0x04) already supports selection:

```rust
// ndn-radio-drivers/src/lib.rs — open_named_radio, else branch
let index = std::env::var("NDN_USB_INDEX").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
let d = Arc::new(Rtl8812auBackend::open_nth(index)?.with_format(fmt));
```

So `NDN_USB_INDEX` works for `rtl8812au` but is silently ignored for `rtl8822e`
(`0xa81a`) — which is exactly the PID our RTL8812EU dongles present.

## Requests (in priority order)
1. **Honor an index in the 88xx path.** Teach `open_pid` (or a new `open_pid_nth`)
   to skip to the Nth match, reading `NDN_USB_INDEX` like the 8812au path — so the two
   backends behave consistently:
   ```rust
   pub fn open_pid_nth(want_pid: u16, index: usize) -> Result<Self, FaceError> {
       let mut seen = 0;
       for device in Context::new()?.devices()?.iter() {
           let d = device.device_descriptor()?;
           if d.vendor_id() == REALTEK_VID && d.product_id() == want_pid {
               if seen == index { return Self::claim(device); }
               seen += 1;
           }
       }
       Err(… "no Realtek 0bda:{want_pid:04x} at index {index}")
   }
   ```
2. **Prefer a *stable* selector over enumeration index.** libusb enumeration order is
   not guaranteed stable across reboots/hotplug. A selector by **USB bus:port** (e.g.
   `1-1.4`) or by the dongle's **MAC** would be robust; index is a fine first cut.
3. **Surface it in `RadioDeviceConfig`, not just an env.** Today `RadioDeviceConfig`
   has only `driver / channel / interface / tx-power`, so even the existing 8812au
   `NDN_USB_INDEX` isn't reachable declaratively — it's env-only. A first-class field
   (`usb-index`, or better `usb-addr = "1-1.4"` / `mac = "…"`) lets a `[[face.radios]]`
   entry pin a specific radio in the TOML, which is what a per-node forwarder config
   needs:
   ```toml
   [[face.radios]]
   driver   = "rtl8822e"
   channel  = 149
   usb-addr = "1-1.4"   # dedicate the spare dongle; leave 1-1.1 on the kernel Wi-Fi
   ```
4. **Coexistence with a kernel-driven sibling.** A short doc note on the expected
   interaction when one identical dongle stays kernel-managed (`rtl88x2eu`) while the
   other is claimed via libusb monitor mode — and whether the backend should refuse /
   warn if it's about to claim a device that currently has an `operstate=up` netdev
   (guard against grabbing the live link).

## What this unblocks
With any of (1)–(3), the miniMUAS pilot can bring up the radio face on the spare dongle
and run the `forwarder × link` A/B matrix (NFD/ndn-fwd × Wi-Fi/radio) on live nodes
without ever dropping the fleet Wi-Fi mesh. Until then we either (a) claim the active
`mesh0` radio (acceptable only when the Wi-Fi cell is intentionally offline for a
radio-cell test), or (b) carry a local `open_pid_nth` patch.

*(ndn-fwd + ndn-tools already build clean for aarch64 with `radio-libusb` and pass a
rung-0 smoke test: boots, NFD-compatible `/localhost/nfd` management, registers app
prefixes with `security.profile="disabled"` and no PIB. The only blocker to radio rungs
is the device-selection gap above.)*

---

## Resolution (ndn-workspace, 2026-08-12)

All four requests shipped; the selector was made **backend-agnostic** rather than USB-named, so a
future serial/SPI/etc. backend reuses the same config surface.

1. **Index + selection on the 88xx path.** `LibUsbRtl88xxBackend::open_pid_select(pid, &sel)` and
   `open_monitor_pid_select(pid, &sel, channel)` (and the 8812au's `open_select(&sel)`) now take a
   `DeviceSelect`. `open_named_radio` reads it from the environment for *both* branches, so the
   88xx path honours selection exactly like the 8812au path did. `open_pid` still means "first".
2. **Stable selector.** `DeviceSelect::Addr("1-1.4")` matches by USB **bus\:port** (the sysfs device
   path), stable across reboots; `DeviceSelect::Index(n)` remains as the non-stable first cut. Both
   live in the new shared `ndn_radio_drivers::usb_select` module used by both Realtek backends.
3. **First-class, backend-agnostic config field.** `RadioDeviceConfig` gains **`address`** (not
   `usb-*`): a device string each driver interprets — the Realtek USB drivers read `"1-1.4"` or
   `"#<index>"`; a serial backend would read a path. `ndn-fwd`'s `radio_face.rs` parses it via
   `DeviceSelect::parse` for both Realtek builders. So the TOML is:
   ```toml
   [[face.radios]]
   driver  = "rtl8822e"
   channel = 149
   address = "1-1.4"    # dedicate the spare dongle; leave 1-1.1 on the kernel Wi-Fi
   ```
   Env equivalents: `NDN_RADIO_DEV` (generic, parsed) or the back-compat `NDN_USB_ADDR` /
   `NDN_USB_INDEX`.
4. **Coexistence guard.** Before claiming, `check_live_link` reads the target device's kernel netdev
   `operstate` from sysfs: if `up` it **warns** (pointing at `address`/`NDN_USB_ADDR`) and, with
   `NDN_GUARD_LIVE_LINK=1`, **refuses** — so an automated bring-up never silently drops the live mesh.
   Claiming detaches only *that* device's kernel driver, so the sibling keeps the mesh; full notes
   (including that release is not a transparent self-reversing borrow) are in the `usb_select` module
   docs. Warn-by-default keeps the single-radio case working without opt-out.

Also fixed in passing: `radio_face.rs` still imported the `WifiRadio` trait that #83 removed (folded
into `FrameIo`), which had broken the `radio-libusb` build; swapped to `Arc<dyn FrameIo>`. `ndn-fwd`
`cargo check --features radio-libusb` is green again.

Touched: `ndn-radio-drivers` (`usb_select.rs` new, `libusb_rtl88xx.rs`, `rtl8812au.rs`, `lib.rs`),
`ndn-config` (`RadioDeviceConfig.address`), `ndn-face-monitor-wifi` (re-export `DeviceSelect`),
`ndn-fwd` (`radio_face.rs`). Unit tests cover the `parse`/match logic; the sysfs guard is Linux-only
and a no-op elsewhere.
