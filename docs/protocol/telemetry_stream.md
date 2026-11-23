# Telemetry stream protocol, version 1

The telemetry stream carries **downlinked telemetry** from `sim_server` to console clients (the GNC console at `/gnc/`). Every packet is a CCSDS Space Packet built by the flight computer and passed through the comms model (`DownlinkModel`: bandwidth, latency, loss), so it is exactly what a ground station would receive. It never carries truth. Packet contents are defined by the telemetry dictionary, `telemetry/dictionary.json` (documented in `docs/telemetry/gnc_packets.md`, generated from it).

This is the live downlink feed of the custom ground segment (`docs/architecture.md`, build order item 48). The ground service (planned) will add archive-backed history and replay on top of the same frames.

## Transport

- WebSocket endpoint `/ws/telemetry` on the same port as the truth stream (default 8080).
- Binary frames carry telemetry; text frames carry JSON control messages.
- Clients must ignore unknown JSON `type` values and unknown binary frame types.
- Client to server messages are ignored (a console cannot restart a run). TODO(phase 10, item 48): sessions, roles and commanding.

## Binary frame: ground frame (type 1)

One ground frame per received CCSDS packet. A 24-byte ground header, **little-endian** (like the truth stream), then the CCSDS packet bytes unchanged (big-endian inside, per the dictionary).

| Offset | Type | Field | Notes |
|---:|---|---|---|
| 0 | u8[4] | magic | ASCII `STLM` (0x53 0x54 0x4C 0x4D) |
| 4 | u16 | protocolVersion | 1 |
| 6 | u16 | frameType | 1 = telemetry packet |
| 8 | u32 | runId | same run id as the truth stream |
| 12 | u16 | packetLength | bytes of CCSDS packet that follow |
| 14 | u16 | reserved | 0 |
| 16 | f64 | groundReceiveTime | sim time when the ground received the packet [s] |
| 24 | u8[packetLength] | packet | CCSDS Space Packet |

Frame size = 24 + packetLength. Latency of a packet = groundReceiveTime - onboard time (from its CUC secondary header).

## JSON messages, server to client

`hello`, sent once when the WebSocket opens:

```json
{
  "type": "hello",
  "stream": "telemetry",
  "protocol": 1,
  "runId": 3,
  "timeScale": 1.0,
  "apids": { "fsw_status": 256, "mission_config": 257, "navigation": 272,
             "guidance_control": 288, "actuator_feedback": 304, "gnss": 320 }
}
```

Right after `hello`, the server replays its **current value table**: the latest ground frame of each APID for the current run, so a console that joins mid-flight fills at once. Replayed frames are ordinary ground frames (a client de-duplicates by APID and sequence count if it cares).

`run_started`, sent when a new run begins: `{ "type": "run_started", "runId": 4 }`. Clients clear their history when `runId` changes.

## Delivery

Each client has its own bounded queue (about 3 s of frames). A slow or stalled client loses its own oldest frames; it never delays the sim, the truth stream or other clients. Lost frames show up as gaps in the per-APID sequence counts, exactly like lost packets on the link, and consoles must treat them that way.

## Versioning

Any change to the ground header increments `protocolVersion`. Changes to packet contents are versioned in the telemetry dictionary.
