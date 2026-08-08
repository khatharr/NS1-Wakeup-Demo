# Switch 1 Wake-up Protocol

[日本語](wakeup-protocol.md) | English

Controllers that wake the original Nintendo Switch, referred to as Switch 1 in this project, from sleep perform a dedicated communication sequence before the normal BR/EDR paging procedure.
This sequence uses a wake-up access code calculated from the BD_ADDR of the target Switch.

In Phase 1, the controller repeatedly transmits a wake-up code.
In Phase 2, it receives connection information from the Switch.
In Phase 3, it transmits its own BD_ADDR.
The Switch wakes from sleep when the controller initiates an ACL link using the LT_ADDR and clock information obtained in Phase 2.

## Terminology

| Term | Meaning |
|---|---|
| LAP | A 24-bit value consisting of the lower three bytes of the BD_ADDR |
| Sync word | A 64-bit value used in a BR/EDR access code |
| Wake-up access code | An access code produced by inverting the lower 32 bits of the standard access code's sync word |
| Wake-up code | A 4-byte value transmitted in the Phase 1 payload |

The wake-up access code and wake-up code are separate values.
The former is a 72-bit access code used for packet synchronization.
The latter is 4 bytes of payload data containing the same value as the lower 32 bits of that access code, arranged in transmission byte order.

## BR/EDR Access Code

A BR/EDR access code consists of 72 bits.

| Field | Length |
|---|---:|
| Preamble | 4 bits |
| Sync word | 64 bits |
| Trailer | 4 bits |

The sync word is calculated from the LAP of the target device.
The upper 3 bytes of the BD_ADDR are not used in the calculation, so BD_ADDR values with the same LAP produce the same sync word.

```text
54:f7:08:90:34:25 -> LAP = 0x903425
ef:4a:c5:1a:00:ee -> LAP = 0x1A00EE
```

The calculation uses the following values defined by the Bluetooth specification.

```text
PN        = 0x83848D96BBCC54FC
GENERATOR = 0x585713DA9
```

`GENERATOR` is the Bluetooth BCH generator polynomial `0x37CD0EB67` multiplied by `D + 1` as a polynomial over GF(2).

```text
GENERATOR = (0x37CD0EB67 << 1) XOR 0x37CD0EB67
```

The sync word `S` is calculated as follows.
Addition and subtraction over GF(2) are performed using XOR.

1. Select a 6-bit Barker sequence according to the most significant bit of the LAP, then derive the 30-bit value `U` used as the input to BCH encoding.

   ```text
   LAP bit23 = 0: U = 0x2C000000 OR LAP
   LAP bit23 = 1: U = 0x13000000 OR LAP
   ```

2. XOR the upper 30 bits of `PN`.

   ```text
   X = U XOR (PN >> 34)
   ```

3. Shift `X` left by 34 bits, divide it by `GENERATOR` as a polynomial over GF(2), and XOR the remainder `R` into the lower 34 bits.

   ```text
   Z = X << 34
   R = Z mod GENERATOR
   C = Z XOR R
   ```

4. XOR the 64-bit `PN` to obtain the sync word.

   ```text
   S = C XOR PN
   ```

The preamble and trailer are determined from the bits at both ends of the sync word in transmission order.

```text
S bit0  = 1: preamble = 1010
S bit0  = 0: preamble = 0101
S bit63 = 1: trailer  = 0101
S bit63 = 0: trailer  = 1010
```

## Wake-up Access Code

The wake-up access code inverts the lower 32 bits of the standard sync word `S`.

```text
P34        = S AND 0x3FFFFFFFF
P34_wakeup = P34 XOR 0xFFFFFFFF
```

`P34` contains the lower 34 bits of the sync word. In `P34_wakeup`, the lower 32 bits are inverted, while the upper 2 bits remain unchanged.
When `P34_wakeup` and the LAP are configured in the Bluetooth controller, the controller generates the 72-bit wake-up access code, including the preamble and trailer.

## Wake-up Code

The wake-up code is produced by inverting the lower 32 bits of the sync word `S` and arranging the result as 4 bytes in little-endian order.

```text
W = (NOT S) AND 0xFFFFFFFF
wakeup_code = little_endian(W)
```

The Phase 1 payload repeatedly contains the 32-bit value obtained by rotating `W` right by 10 bits, encoded in little-endian order.

```text
phase1_word = rotate_right(W, 10)
phase1_payload = repeat(little_endian(phase1_word))
```

Examples of these calculation results are shown below.

| BD_ADDR | 32-bit value `W` | 4-byte wake-up code | 72-bit wake-up access code |
|---|---|---|---|
| `55:55:55:55:55:55` | `0x79F98210` | `10 82 F9 79` | `AB155555579F98210A` |
| `aa:aa:aa:aa:aa:aa` | `0xFAD69689` | `89 96 D6 FA` | `54EAAAAA9FAD696895` |
| `54:f7:08:90:34:25` | `0x0CB13E3F` | `3F 3E B1 0C` | `54E40D0940CB13E3F5` |
| `ef:4a:c5:1a:00:ee` | `0xAF3BC02D` | `2D C0 3B AF` | `AB06803B9AF3BC02D5` |

## Wake-up Sequence

All three phases use the same wake-up access code.

| Phase | Sender | Content | Purpose |
|---|---|---|---|
| 1 | Controller | Repeated wake-up code | Requests a response from the sleeping Switch |
| 2 | Switch | 17 bytes containing the LT_ADDR, Switch BD_ADDR, and native clock | Provides the information required to initiate an ACL link |
| 3 | Controller | The controller's own BD_ADDR followed by 6 bytes of zero padding | Provides the controller's address to the Switch |

### Phase 1

The controller transmits raw RF packets containing repeated copies of the wake-up code at intervals of 3.75 ms.
RF captures of third-party controllers show that either DH3 or DH5 is used and that the payload is not whitened.
This is thought to reproduce arbitrary RF packet transmission using Bluetooth packets.

### Phase 2

The Switch responds using the wake-up access code generated from `P34_wakeup` and the LAP. This access code differs from the standard BR/EDR access code derived from the same BD_ADDR.
Measurements from RF captures show that the response is transmitted as a DM1 packet with a 17-byte payload.

| Item | Value |
|---|---|
| Access code | `P34_wakeup` |
| Packet type | DM1 |
| Packet header | `0x298`, LT_ADDR=0, TYPE=3, FLOW=1, ARQN=0, SEQN=1 |
| Payload length | 17 bytes |
| Whitening | None |
| Transmission interval | 3.75 ms |

The payload fields used to initiate the connection are as follows.

| Byte position | Content |
|---|---|
| 0 | The lower 3 bits contain the LT_ADDR |
| 1–6 | The Switch BD_ADDR, stored least-significant byte first |
| 7–12 | Not used by this project |
| 13–16 | The Switch native clock |

The native clock is reconstructed using the following expression.

```text
packed_clock = payload[13]
             OR (payload[14] << 8)
             OR (payload[15] << 16)
             OR ((payload[16] AND 0x1F) << 24)

remote_clock = packed_clock >> 2
```

The controller calculates the difference between `remote_clock` and the local clock at which the response was received, then uses this difference as the clock offset for the ACL link.

### Phase 3

The controller transmits Phase 3 634 us after the most recently received Phase 2 response.
Measurements from RF captures show that Phase 3 is a DM1 packet with a 12-byte payload.

| Item | Value |
|---|---|
| Access code | `P34_wakeup` |
| Packet type | DM1 |
| Packet header | `0x298`, LT_ADDR=0, TYPE=3, FLOW=1, ARQN=0, SEQN=1 |
| Payload header | `0x67`, length 12, FLOW=1, LLID=3 |
| Payload | 6 bytes of the controller BD_ADDR, stored least-significant byte first, followed by 6 bytes of zero padding |
| Whitening | None |
| Transmission time | 634 us after the most recent Phase 2 response |

A short RF burst is observed from the Switch after Phase 3 and is used by this project as an indication that Phase 3 was accepted.
The following ACL link initiation procedure is still required to wake the Switch from sleep.

### Initiating the ACL Link

The controller initiates an ACL link using the LT_ADDR, Switch BD_ADDR, and native clock obtained in Phase 2.
The Switch wakes from sleep as the ACL link is initiated, after which the normal BR/EDR connection procedure continues.
Receiving Phase 3 alone does not complete the wake-up process.

## Reference Timing

The following reference values were obtained from RF captures of Nintendo and third-party controllers.

| Item | Reference value |
|---|---:|
| Phase 1 transmission interval | 3.75 ms |
| Time from the start of Phase 1 transmission to the Phase 2 response | 1604–2386 us |
| Duration of consecutive Phase 2 responses | ~40 ms |
| Phase 2 response interval | Approximately 320 ms |
| Time from the Phase 2 response to Phase 3 transmission | 634 us |
| Time from Phase 3 to the Switch response | 1870 us |

## Reference Implementation

The three phases are transmitted and received using the ESP32 Connectionless Slave Broadcast feature.
Whitening applied by the ESP32 Bluetooth controller is neutralized either by applying the same whitening sequence to the transmit and receive data in advance, or by disabling whitening through the controller's test mode.
The 17-byte Phase 2 payload is obtained from the ACL receive context.
The receive clock and in-slot time recorded by the hardware are used to schedule the Phase 3 transmission.

Initiating an ACL link from the Phase 2 information wakes the Switch.
However, because the current implementation cannot receive L2CAP packets from the Switch, it disconnects that ACL link and starts a normal paging procedure.

The calculation is implemented in [wakeup_code.c](../src/wake/wakeup_code.c), and payload generation is implemented in [wake_payload.c](../src/wake/wake_payload.c).
