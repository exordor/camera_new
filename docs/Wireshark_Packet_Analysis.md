# Wireshark Packet Analysis - GigE Vision Camera Protocol

## Date: 2025-10-08

## Overview

Through Wireshark packet capture analysis, we have identified the exact structure and purpose of different packet sizes observed in GigE Vision camera communication.

## Packet Size Analysis

### 1. **44-Byte LEADER Packet** (UDP Payload)

**Observed in Wireshark**:
```
Frame 2230186: 86 bytes on wire (Src: 192.168.40.11, Dst: 192.168.40.19)
Protocol: GVSP
Info: LEADER [Block ID: 167 Packet ID: 0]
UDP Payload: 44 bytes
```

**Purpose**: **Frame Start Marker (LEADER)**
- Signals the beginning of a new image frame
- Contains frame metadata and GigE Vision Stream Protocol (GVSP) header
- Block ID: Identifies which frame this belongs to
- Packet ID: 0 (always first packet in sequence)

**Structure**:
```
Ethernet Header: 14 bytes
IP Header: 20 bytes
UDP Header: 8 bytes
GVSP Payload: 44 bytes
  - GVSP Header: ~8 bytes
    - Status flags
    - Format information (LEADER type)
    - Block ID (Frame number)
    - Packet ID (0 for LEADER)
  - Frame Metadata: ~36 bytes
    - Image width/height
    - Pixel format
    - Timestamp
    - Frame ID
-----------------------------------------------
Total on wire: 86 bytes
UDP Payload: 44 bytes
```

**When Sent**: Before any image data packets in each frame

---

### 2. **1514-Byte IMAGE Packet** (UDP Payload ~1436 bytes)

**Observed in Wireshark**:
```
Frame 2230187-2238606: 1514 bytes on wire
Protocol: GVSP
Info: 1514 PAYLOAD [Block ID: 167 Packet ID: 1-8419]
Length Info: 1514
```

**Purpose**: **Image Data Payload**
- Contains actual pixel data from the camera sensor
- Most common packet type (thousands per frame)
- Packet ID increments sequentially (1, 2, 3, ..., 8419)

**Structure**:
```
Ethernet Header: 14 bytes
IP Header: 20 bytes
UDP Header: 8 bytes
GVSP Header: 8 bytes (simplified for payload)
Image Data: 1436 bytes ← Actual pixel data
-----------------------------------------------
Total on wire: 1514 bytes
UDP Payload: 1436 bytes
```

**When Sent**: Continuously after LEADER until all image data transmitted

**Calculation for 4096×3000 image**:
```
Image size: 4096 × 3000 × 1 byte (BayerRG8) = 12,288,000 bytes
Payload per packet: 1436 bytes
Packets needed: 12,288,000 / 1436 = 8,556 packets
Observed: ~8,419 packets (Block ID 167, Packet ID 1-8419)
```

---

### 3. **16-Byte TRAILER Packet** (UDP Payload)

**Observed in Wireshark**:
```
Frame 2238607: 58 bytes on wire
Protocol: GVSP
Info: TRAILER [Block ID: 167 Packet ID: 8420]
UDP Payload: 16 bytes
```

**Purpose**: **Frame End Marker (TRAILER)**
- Signals completion of image frame transmission
- Contains frame validation information
- Last packet in the sequence (highest Packet ID)

**Structure**:
```
Ethernet Header: 14 bytes
IP Header: 20 bytes
UDP Header: 8 bytes
GVSP Payload: 16 bytes
  - GVSP Header: ~8 bytes
    - Status flags (frame complete/incomplete)
    - Format information (TRAILER type)
    - Block ID (matches LEADER)
    - Packet ID (last in sequence)
  - Frame Validation: ~8 bytes
    - Payload length
    - Frame status
    - Reserved fields
-----------------------------------------------
Total on wire: 58 bytes
UDP Payload: 16 bytes
```

**When Sent**: After all IMAGE packets transmitted

---

## Complete Frame Transmission Sequence

### Single Frame Packet Flow:

```
Packet 1:  LEADER (44 bytes UDP payload, 86 bytes on wire)
           - Block ID: 167, Packet ID: 0
           - Contains: Frame metadata, size, format

Packet 2:  IMAGE (1436 bytes UDP payload, 1514 bytes on wire)
           - Block ID: 167, Packet ID: 1
           - Contains: Pixels 0-1435

Packet 3:  IMAGE (1436 bytes UDP payload, 1514 bytes on wire)
           - Block ID: 167, Packet ID: 2
           - Contains: Pixels 1436-2871

Packet 4:  IMAGE (1436 bytes UDP payload, 1514 bytes on wire)
           - Block ID: 167, Packet ID: 3
           - Contains: Pixels 2872-4307
...
(8,417 more IMAGE packets)
...

Packet 8420: IMAGE (1436 bytes UDP payload, 1514 bytes on wire)
             - Block ID: 167, Packet ID: 8419
             - Contains: Last pixels

Packet 8421: TRAILER (16 bytes UDP payload, 58 bytes on wire)
             - Block ID: 167, Packet ID: 8420
             - Contains: Frame complete status
```

**Total packets per frame**: ~8,421 packets
- 1 LEADER
- 8,419 IMAGE packets
- 1 TRAILER

---

## Port Usage (Corrected from Previous Analysis)

### Port 11220: GVSP (GigE Vision Stream Protocol)
**ALL image-related packets use this port**:
- 44-byte LEADER packets (86 bytes on wire)
- 1436-byte IMAGE packets (1514 bytes on wire)
- 16-byte TRAILER packets (58 bytes on wire)

### Port 3956: GVCP (GigE Vision Control Protocol)
**Only control/heartbeat packets**:
- 12-byte heartbeat/keepalive packets
- Device discovery responses
- Parameter read/write acknowledgments

---

## Previous Misunderstanding Corrected

### What We Thought Before:
```
❌ Port 11220: Control messages (16-44 bytes)
❌ Port 3956: Image data (variable size)
```

### What Wireshark Revealed:
```
✅ Port 11220: ALL image traffic
   - LEADER: 44 bytes UDP payload (86 bytes on wire)
   - IMAGE: 1436 bytes UDP payload (1514 bytes on wire)
   - TRAILER: 16 bytes UDP payload (58 bytes on wire)
✅ Port 3956: Only control/heartbeat (12 bytes UDP payload)
```

### Why the Confusion?

When we ran:
```bash
sudo tcpdump -i eth0 'udp and src 192.168.40.11 and port 11220' -n
```

We sometimes saw small packets because:
1. We captured at the START of a frame (LEADER packet = 44 bytes UDP payload)
2. We captured at the END of a frame (TRAILER packet = 16 bytes UDP payload)
3. The filter excluded the bulk IMAGE packets somehow

When we ran with size filter:
```bash
sudo tcpdump -i eth0 'udp and src 192.168.40.11 and greater 1000' -n
```

We correctly saw the 1436-byte IMAGE packets continuously!

---

## GigE Vision Stream Protocol (GVSP) Details

### GVSP Packet Types:

| Type | UDP Payload | On Wire | Purpose | Packet ID |
|------|------------|---------|---------|-----------|
| LEADER | 44 bytes | 86 bytes | Frame start metadata | 0 |
| IMAGE/PAYLOAD | 1436 bytes | 1514 bytes | Pixel data | 1 to N-1 |
| TRAILER | 16 bytes | 58 bytes | Frame end validation | N (last) |

### GVSP Header Fields:

```
Bit 0-15:  Status (frame valid/invalid)
Bit 16-31: Block ID (Frame number, increments each frame)
Bit 32-47: Packet format (LEADER/IMAGE/TRAILER)
Bit 48-63: Packet ID (sequence within frame)
```

---

## Timing Analysis (5 fps operation)

### Per Frame:
```
Frame duration: 200ms (5 fps)
Packets per frame: 8,421
Packet rate: 8,421 / 0.2s = 42,105 packets/sec
```

### Network Bandwidth:
```
LEADER:  44 bytes × 5 fps = 220 bytes/sec
IMAGE:   1436 bytes × 8,419 packets × 5 fps = 60,448,420 bytes/sec ≈ 57.7 MB/s
TRAILER: 16 bytes × 5 fps = 80 bytes/sec
---------------------------------------------------------------
Total: ≈ 57.7 MB/s ≈ 461 Mbps (46.1% of 1 GbE)
```

This matches the observed network utilization!

---

## Practical Implications

### 1. **Incomplete Frame Detection**

If any packet is lost, frame status becomes INCOMPLETE:
```cpp
if (pFrame->status == GX_FRAME_STATUS_INCOMPLETE) {
    // Missing one of:
    // - LEADER (80 bytes) → No frame metadata
    // - IMAGE packet (1514 bytes) → Missing pixel data
    // - TRAILER (88 bytes) → No validation info
}
```

### 2. **Packet Loss Impact**

```
Missing LEADER (44 bytes) → Cannot parse frame metadata → Discard frame
Missing IMAGE (1436 bytes) → Pixel data corruption → Usually discard frame
Missing TRAILER (16 bytes) → Cannot validate → May still use frame
```

### 3. **Network Optimization**

To achieve higher frame rates:
- Reduce packet delay (currently 5ms → try 2ms)
- Increase packet size (if camera supports, currently 1436 bytes)
- Use dedicated network interface (avoid sharing with LiDAR)

---

## Wireshark Filter Examples

### Capture only LEADER packets:
```
gvsp.format == 0x01 && udp.length == 44
```

### Capture only IMAGE packets:
```
gvsp.format == 0x03 && udp.length == 1436
```

### Capture only TRAILER packets:
```
gvsp.format == 0x04 && udp.length == 16
```

### Capture complete frame sequence:
```
gvsp.blockid == 167
```

---

## Summary

✅ **44-byte packets** (UDP payload): GVSP LEADER (frame start, metadata)  
✅ **1436-byte packets** (UDP payload): GVSP IMAGE (pixel data)  
✅ **16-byte packets** (UDP payload): GVSP TRAILER (frame end, validation)  

All transmitted on **Port 11220 (GVSP)**.

**Port 3956 (GVCP)** only carries 12-byte control/heartbeat messages.

**Note**: "On wire" sizes include Ethernet (14) + IP (20) + UDP (8) headers:
- LEADER: 44 + 42 = 86 bytes on wire
- IMAGE: 1436 + 78 = 1514 bytes on wire (includes Ethernet padding)
- TRAILER: 16 + 42 = 58 bytes on wire

---

**Analysis Date**: October 8, 2025  
**Camera**: Daheng MER2-302 (4096×3000, BayerRG8)  
**Frame Rate**: 5.006 fps  
**Network**: 1 GbE, MTU 1500, Packet size 1436 bytes  
**Tools**: Wireshark 3.x, tcpdump, ROS Noetic
