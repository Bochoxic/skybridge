# MAVLink telemetry over ExpressLRS WiFi

How to get MAVLink telemetry from a Betaflight flight controller to a laptop
over WiFi, using the ExpressLRS TX module's built-in backpack as the relay.

End result: a script on your laptop reading live attitude/sensor data at
20-50 Hz, with no USB cable to the TX module.

```
Drone FC ──MAVLink──> ELRS RX ──2.4GHz──> ELRS TX ──> Backpack ──WiFi/UDP──> laptop
```

---

## What this does and does not give you

**Telemetry works.** Attitude, battery, GPS, whatever the FC emits.

**RC control does NOT work over this path.** The MAVLink tunnel carries
telemetry only. The ELRS TX module never parses MAVLink you send into it —
it forwards the bytes verbatim, and the RX generates its own
`RC_CHANNELS_OVERRIDE` from the OTA channel packets (which come from the
handset's CRSF input). Sending `RC_CHANNELS_OVERRIDE` from a GCS fights
that stream and arrives at a few Hz.

To command the drone from a laptop, send **CRSF** to the TX module's RC
input on a separate serial port. That is a different path and can run at
the same time as this one — see [section 6](#6-rc-control-over-crsf-and-free-crsf-telemetry).

---

## Requirements

| Component | Requirement |
|---|---|
| ELRS TX module | ESP-based, with a TX Backpack (e.g. RadioMaster Ranger Micro) |
| ELRS RX | ESP-based |
| Backpack firmware | **1.5.0 or later** — earlier builds have no MAVLink support |
| FC firmware | Betaflight with MAVLink support |
| Laptop | Python 3, `pymavlink` |

Vendor backpack builds (e.g. `DUPLETX_TX_Backpack`) are often too old.
Check the version before anything else — see step 2.

---

## 1. Betaflight

In the Betaflight Configurator:

1. **Ports tab** — on the UART wired to your ELRS receiver, set the
   Telemetry Output protocol to **MAVLink**.
2. **Baud rate** — `460800`. This is what ELRS expects; other values
   silently fail.
3. Set the MAVLink telemetry rate. **10-20 Hz is plenty** for attitude
   monitoring. Higher rates are the most common cause of lag (see
   Troubleshooting).
4. Save and reboot.

---

## 2. ELRS TX Backpack firmware

The backpack must be running a MAVLink-capable build. Check what you have
by browsing to the backpack's web page (power the module without the
handset connected to bring up its WiFi, then visit `http://10.0.0.1`).

Look at the version string:

- `ExpressLRS TX Backpack` rev **1.5.0+** → good
- `DUPLETX_TX_Backpack` rev 1.3.0, or any build whose web page has **no
  "MavLink Configuration" section** → must be reflashed

### Reflashing

Use the **ExpressLRS Configurator**:

1. Select **Backpack** in the left menu
2. Choose the latest Backpack release (≥ 1.5.0)
3. Device target: your module (e.g. `radiomaster.txbp.ranger-micro`)
4. **Binding phrase**: must match your TX module exactly
5. **Flashing method**: `WiFi` if the backpack is reachable on your
   network, otherwise `Passthrough` with a USB cable to the module

Wait for the LED to resume blinking before removing power. An interrupted
flash requires USB/serial recovery.

> Reflashing **resets the backpack config**, including saved WiFi
> credentials and the telemetry mode. Expect to redo step 3 and rejoin
> your network afterwards.

---

## 3. ELRS module settings (Lua script on the handset)

In the ELRS Lua script:

| Setting | Value |
|---|---|
| Link mode | **MAVLink** |
| Backpack → Telemetry | **WiFi** |
| Packet rate | **F1000** or **333Hz Full** (see note) |

**Packet rate matters more than you would expect.** MAVLink frames are
large and share airtime with RC packets (MAVLink mode forces a 1:2
telemetry ratio). Low rates like 50Hz leave too few slots, so telemetry
queues up and arrives seconds late. Counter-intuitively, **higher packet
rates reduce lag** for bench/short-range work. Lower rates only make sense
when you actually need the range.

Setting `Telemetry → WiFi` sends an MSP message to the backpack that
enables MAVLink forwarding and reboots it.

---

## 4. Network

The backpack can either create its own AP or join your network.

**Own AP** (simplest):
- SSID `ExpressLRS TX Backpack XXXXXX`, password `expresslrs`
- Backpack is at `10.0.0.1`, you get a `10.0.0.x` address
- No internet while connected

**Your home network** (STA mode): join it from the backpack's web page
("Join Network"). It then resolves as `elrs_txbp.local`.

### Verify it is actually forwarding

This is the single most useful check:

```bash
curl -s --compressed http://elrs_txbp.local/mavlink
```

```json
{"enabled":true,
 "counters":{"packets_down":2541,"packets_up":74,"drops_down":0,"overflows_down":1},
 "ports":{"listen":14555,"send":14550},
 "ip":{"gcs":"192.168.0.121"},
 "protocol":"UDP"}
```

| Field | Meaning |
|---|---|
| `enabled` | **Must be `true`.** If false, MAVLink forwarding is off — see Troubleshooting |
| `packets_down` | Telemetry arriving from the drone. Should climb steadily |
| `packets_up` | Packets received from your laptop |
| `overflows_down` | Queue discards. Should stay near zero |
| `ip.gcs` | Your laptop's IP once it has announced itself |

---

## 5. Run the reader

```bash
python3 wifi_telem.py
```

Live single-line readout:

```
roll=   -0.60deg  pitch=  -11.00deg  yaw= +359.60deg  | p=+0.0 q=+0.0 r=+0.0 deg/s | 19.8Hz
```

### Ports

The backpack **sends to 14550** and **listens on 14555**. These are
different, which trips up most setups:

```
laptop :14550  <──telemetry──  backpack
laptop         ──heartbeats──> backpack :14555
```

The heartbeats matter. Until the backpack receives a packet from you it
does not know your IP (`"gcs": "IP UNSET"`) and falls back to subnet
broadcast, which many routers drop. The script sends GCS heartbeats to
14555 once a second so the backpack latches onto your address and
unicasts.

---

## 6. RC control over CRSF (and free CRSF telemetry)

The MAVLink tunnel above carries telemetry only. To **command** the drone
from the laptop, send CRSF to the module's RC input on a serial port.
This runs alongside the WiFi telemetry with no contention — the two share
no resource.

As a bonus, CRSF is bidirectional: the module sends telemetry back on the
same wire for free, with no configuration. That stream is **CRSF
telemetry, not MAVLink** — a narrower message set (attitude, battery,
baro, flight mode), but lower latency since it comes straight down the
control link.

### Wiring

A USB-serial adapter (CP210x, CH340, FTDI) from the laptop to the
module's serial pins. It shows up as `/dev/ttyUSB0`.

> The module's own USB-C port is **not** this path — that is
> `/dev/ttyACM*` and is used for the MAVLink tunnel and firmware updates.

> Avoid the JR-bay S.Port pin unless you have an inverter: that signal is
> inverted (idle low) and a plain USB-UART adapter cannot drive it.

### Module config

`serial_rx` / `serial_tx` must point at the pins your adapter is wired to.
On a Ranger Micro the stock target is:

```json
"serial_rx": 13,   // JR bay (handset)
"serial_tx": 13
```

For laptop control over UART0, set both to **`3, 1`**.

This is a *move*, not an addition — the JR bay stops being the RC input,
so the handset can no longer command while the laptop does. The module
still needs power: either the handset (sticks ignored) or the XT30
connector.

### Baud rate: 115200

Use **115200** on this path — it is what the USB-serial adapter into the
module's UART0 pins runs at, and it is `crsf_test.py`'s default:

```bash
python3 crsf_test.py            # /dev/ttyUSB0 @ 115200
```

> The CRSF *standard* rate is 420000, which is what the JR-bay RC pins use
> between a handset and a module. That rate does **not** work here. If you
> pick the wrong one, every byte is garbled, the CRC fails, and the module
> silently discards all RC frames — with no error anywhere.

> **Telemetry arriving does not prove your RC is accepted.** The downlink
> is generated by the module independently of whether it accepts your
> uplink frames. Both directions fail independently — verify RC in the
> Betaflight Receiver tab, not by seeing telemetry.

### Betaflight config

1. **Ports tab** — on the ELRS receiver's UART, set Receiver protocol to
   **CRSF** (serial-based receiver)
2. **Receiver tab** — Receiver Mode: `Serial (via UART)`, Provider: `CRSF`
3. **Failsafe** — check your failsafe channel values. A channel whose
   failsafe is set to **1500** looks identical to a live centered stick,
   so the Receiver tab appears to work while nothing is actually getting
   through. Set failsafe values somewhere distinctive (or to `Auto`) while
   debugging so real input is obvious.

### Channel mapping

`crsf_test.py` sends 16 channels in CRSF units, not microseconds:

| CRSF value | Microseconds | Meaning |
|---|---|---|
| `CH_MIN` = 172 | 1000 | low |
| `CH_MID` = 992 | 1500 | centre |
| `CH_MAX` = 1811 | 2000 | high |

Default layout: 1 roll, 2 pitch, 3 throttle (MIN), 4 yaw, 5 AUX1/ARM
(MIN = disarmed). `us_to_crsf()` converts if you prefer to think in
microseconds.

### Verifying

Open the Betaflight **Receiver tab** and confirm the channel bars follow
what the script sends. `crsf_test.py` sweeps yaw as a slow sine, which is
easy to spot — a moving bar means the uplink is genuinely accepted.

---

## Troubleshooting

### `"enabled": false`

MAVLink forwarding is off. The service is read **once at boot** and cannot
be changed at runtime, so the `/setmavlink` HTTP endpoint alone will not
fix it.

"Enable Backpack WiFi" in Lua starts the backpack in *firmware update*
mode — you get a working web page and a **disabled** MAVLink service.
These are mutually exclusive modes. You need `Backpack → Telemetry → WiFi`,
which pushes an MSP message that sets the service and reboots.

If it stays false, toggle Telemetry to `Off`, back out so it commits, then
set it to `WiFi` again. If WiFi never comes up on its own afterwards,
suspect a **binding phrase mismatch** between backpack and module — the
module cannot push settings to a backpack it is not paired with.

### Nothing arrives, but `packets_down` is climbing

Data reaches the backpack but not you.

- Check `ip.gcs`. If `IP UNSET`, your heartbeats are not landing — confirm
  you are sending to the **listen** port (14555), not 14550
- Firewall: `sudo ufw allow 14550/udp`
- Confirm packets are physically arriving:
  ```bash
  sudo tcpdump -i <iface> -n udp and host <backpack-ip>
  ```
  If tcpdump sees inbound packets but the script does not, it is the
  firewall. If it sees nothing inbound, the backpack is not sending.

### pymavlink's `udpin` receives nothing

`mavutil.mavlink_connection("udpin:0.0.0.0:14550")` locks onto one peer
address and **silently drops** packets arriving from port 14555. Manage
the socket directly and feed `MAVLink.parse_buffer()` yourself — this is
what `wifi_telem.py` does.

### Seconds of lag

Almost always bandwidth. Check whether `overflows_down` is climbing fast.

1. **Raise the ELRS packet rate** to F1000 or 333Hz Full. More slots per
   second means less fragmentation and queuing
2. **Lower the Betaflight telemetry rate**. 10-20 Hz is enough for
   attitude; 40+ Hz through a constrained link causes backup

If the handset display lags too, the bottleneck is upstream of WiFi —
it is the radio link, not your network.

### No HEARTBEAT

Betaflight over ELRS may not emit `HEARTBEAT` at all. Do not block waiting
for one — accept any message as proof of life.

### Backpack vanishes after reboot

If it is not on your network and not broadcasting an AP, it likely booted
into ESP-NOW mode (`StartWiFiOnBoot` false). Re-set `Telemetry → WiFi` on
the handset.
