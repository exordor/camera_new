# Emergency Fix: Camera Cannot Receive UDP Data After Jumbo Frame Configuration

## Problem Diagnosis

**Symptom**: Camera cannot receive UDP data after setting `gev_packet_size` to 8000 bytes.

**Root Cause**: MTU mismatch between PC and camera
- PC MTU: 8966 bytes (supports jumbo frames)
- Camera MTU: 1500 bytes (standard Ethernet)
- Packet size configured: 8000 bytes
- Result: Camera DROPS all packets > 1500 bytes

**Proof**:
```bash
# Small packets work
ping -c 3 192.168.40.11
# Result: ✓ Success

# Large packets fail
ping -c 3 -M do -s 8000 192.168.40.11
# Result: ✗ 100% packet loss
```

## Immediate Solution

### Option 1: Revert to Standard Ethernet (RECOMMENDED)

**Step 1**: Edit launch file to use standard packet size:

```bash
nano ~/catkin_ws_eagrumo/src/catkin_ws_eas/src/galaxy_camera_ros_driver-master/launch/MER2-302.launch
```

Change this line:
```xml
<param name="gev_packet_size" value="8000"/>
```

To:
```xml
<param name="gev_packet_size" value="1500"/>
```

**Step 2**: Reset PC MTU to standard:

```bash
sudo ip link set eth0 down
sudo ip link set eth0 mtu 1500
sudo ip link set eth0 up
```

**Step 3**: Verify connectivity:

```bash
ping -c 3 192.168.40.11
# Should work
```

**Step 4**: Restart camera:

```bash
roslaunch galaxy_camera MER2-302.launch
```

### Option 2: Configure Camera for Jumbo Frames (Advanced)

**Warning**: This requires camera to support jumbo frames AND you need to configure the camera's network settings.

**Step 1**: Check if camera supports jumbo frames:
- Login to camera's web interface (http://192.168.40.11)
- OR use Galaxy camera configuration tool
- Look for MTU or packet size settings

**Step 2**: Set camera MTU to 9000:
- This is camera-specific, consult camera manual
- Many GigE Vision cameras don't expose this setting

**Step 3**: If camera doesn't support jumbo frames, go to Option 1

## Why This Happened

### Network Communication Basics

```
PC sends 8000-byte packet
    ↓
Network switch
    ↓
Camera receives packet
    ↓
Camera checks: Is packet size ≤ my MTU (1500)?
    ↓ NO (8000 > 1500)
    ✗ DROPPED
```

### The MTU Mismatch Problem

| Device | MTU | Can Send | Can Receive |
|--------|-----|----------|-------------|
| PC (eth0) | 8966 | Up to 8966 bytes | Up to 8966 bytes |
| Camera | 1500 | Up to 1500 bytes | Up to 1500 bytes |
| Result | ❌ | Camera drops PC's 8000-byte packets | |

### What Happens with Jumbo Frames

1. **PC sets packet size to 8000 bytes** via `GX_INT_GEV_PACKETSIZE`
2. **SDK sends 8000-byte UDP packets** to camera
3. **Camera's network interface checks MTU**:
   - Camera MTU = 1500 bytes
   - Packet size = 8000 bytes
   - 8000 > 1500 → **DROP PACKET**
4. **PC never receives acknowledgment**
5. **SDK times out** waiting for data
6. **No frames received**

## Quick Fix Commands

### Revert to Working Configuration

```bash
# 1. Stop camera if running
rosnode kill /galaxy_camera

# 2. Reset network to standard MTU
sudo ip link set eth0 down
sudo ip link set eth0 mtu 1500
sudo ip link set eth0 up

# 3. Edit launch file - change packet size to 1500
# (See Option 1 above)

# 4. Rebuild workspace
cd ~/catkin_ws_eagrumo/src/catkin_ws_eas
catkin_make

# 5. Restart camera
roslaunch galaxy_camera MER2-302.launch
```

### Verify Fix

```bash
# Terminal 1: Monitor frame rate
rostopic hz /galaxy_camera/image_raw
# Should show: ~3 Hz

# Terminal 2: Check for errors
rostopic echo /rosout | grep -i error
# Should be clean

# Terminal 3: Monitor network
sudo tcpdump -i eth0 'src 192.168.40.11' -n
# Should see regular packets
```

## Understanding the Camera's Limitations

### Why Camera Doesn't Support Jumbo Frames

Most industrial GigE Vision cameras (including MER2-302) have:

1. **Fixed MTU of 1500 bytes** - Hardware limitation in the camera's network chip
2. **No user-configurable MTU** - Firmware doesn't expose this setting
3. **GigE Vision standard default** - Designed for maximum compatibility

### SDK vs Camera Behavior

```cpp
// Your code sets:
GXSetInt(dev_handle_, GX_INT_GEV_PACKETSIZE, 8000);
// SDK says: "OK, I'll send 8000-byte packets"

// But camera says:
// "My MTU is 1500, I can't receive packets > 1500"
// → Packets dropped
```

This is why SDK doesn't error out - it's a **runtime network problem**, not a software error.

## Recommended Configuration

### For Stable Operation (Use This)

```xml
<!-- MER2-302.launch - WORKING CONFIGURATION -->

<!-- Network parameters -->
<param name="gev_packet_size" value="1500"/>
<param name="gev_packet_delay_us" value="5000"/>
<param name="gev_resend_timeout_ms" value="500"/>
<param name="gev_packet_timeout_ms" value="1000"/>
<param name="gev_block_timeout_ms" value="60000"/>
<param name="gev_socket_buffer_kb" value="8192"/>
```

### Expected Performance

With this configuration:
- **Frame rate**: ~3.08 fps
- **Incomplete frames**: 0%
- **Network stability**: 100%
- **Compatibility**: Works with all network equipment

## Alternative Performance Improvements

Since jumbo frames don't work, try these instead:

### 1. Reduce Packet Delay (Faster Transmission)

```xml
<param name="gev_packet_delay_us" value="2000"/>  <!-- Was 5000 -->
```

**Expected improvement**: 3.08 fps → 5+ fps  
**Risk**: May cause packet collisions with lidar

### 2. Lower Resolution (Fewer Packets)

```xml
<param name="image_width" value="2048"/>   <!-- Was 4096 -->
<param name="image_height" value="1500"/>  <!-- Was 3000 -->
```

**Expected improvement**: 4× faster (12+ fps)  
**Trade-off**: Lower image resolution

### 3. Separate Network Interface (Best Solution)

Add a second network card:
- **eth0**: Lidar only
- **eth1**: Camera only

**Expected result**: Full 5 fps with no packet delay needed

## Troubleshooting Checklist

- [ ] PC MTU = 1500 bytes
- [ ] Launch file: `gev_packet_size = 1500`
- [ ] Camera responds to ping
- [ ] No firewall blocking UDP ports 3956, 11220
- [ ] Camera and PC on same subnet (192.168.40.x)
- [ ] System buffers increased (check `/etc/sysctl.conf`)

## Files to Check/Edit

1. **Launch file**: `~/catkin_ws_eagrumo/src/catkin_ws_eas/src/galaxy_camera_ros_driver-master/launch/MER2-302.launch`
   - Change `gev_packet_size` to 1500

2. **Network interface**: 
   ```bash
   sudo ip link set eth0 mtu 1500
   ```

3. **System logs**: 
   ```bash
   roscd galaxy_camera
   ls -lt ~/.ros/log/latest/galaxy_camera-*.log
   ```

## Summary

**Problem**: Camera cannot receive 8000-byte packets because its MTU is 1500 bytes  
**Solution**: Use standard packet size (1500 bytes) matching camera's capability  
**Performance**: Accept 3.08 fps or use alternative optimizations  
**Lesson**: Always match MTU settings between sender and receiver

---
Date: 2025-10-07  
Status: CRITICAL - MUST FIX BEFORE CAMERA WORKS AGAIN
