# Quick Reference: Camera Packet Size Configuration

## Summary

The camera packet size (frame length) is now configurable via ROS parameters. This allows optimization for different network configurations.

## Current Configuration

- **Network MTU**: 8966 bytes (jumbo frames enabled)
- **Launch file packet size**: 8000 bytes
- **Recommended**: MTU should be at least (packet_size + 28) bytes

## Quick Commands

### 1. Set MTU to 9000 (for 8000-byte packets)
```bash
sudo ip link set eth0 down
sudo ip link set eth0 mtu 9000
sudo ip link set eth0 up
```

### 2. Verify MTU
```bash
ip link show eth0 | grep mtu
```

### 3. Launch camera with custom packet size
```bash
roslaunch galaxy_camera MER2-302.launch gev_packet_size:=8000
```

### 4. Monitor frame rate
```bash
rostopic hz /galaxy_camera/image_raw
```

### 5. Check for incomplete frames
```bash
rosnode info /galaxy_camera | grep -A5 "Subscriptions"
# Watch the ROS logs for frame status reports
```

## Configuration in Launch File

Edit `/src/galaxy_camera_ros_driver-master/launch/MER2-302.launch`:

```xml
<!-- Standard Ethernet (1500 bytes) -->
<param name="gev_packet_size" value="1500"/>

<!-- OR Jumbo Frames (8000 bytes) -->
<param name="gev_packet_size" value="8000"/>
```

## Expected Performance

| Configuration | Packets/Frame | Transmission Time* | Expected FPS |
|--------------|---------------|-------------------|--------------|
| 1500 bytes   | 8,556         | 42.78s            | 3.08 fps     |
| 8000 bytes   | 1,541         | 7.71s             | 17+ fps      |

*With 5ms inter-packet delay

## Important Notes

1. **MTU Requirement**: Network MTU must be ≥ (packet_size + 28) bytes
   - For 8000-byte packets: MTU ≥ 8028 bytes (use 9000)
   - For 1500-byte packets: MTU ≥ 1528 bytes (use 1500)

2. **Switch Support**: All network switches must support jumbo frames

3. **Camera Hardware**: Some cameras may not support large packet sizes

4. **Verify Configuration**: Always check logs for actual packet size used

## Troubleshooting

### Problem: Camera still uses 1500 bytes despite setting 8000

**Solution**: Camera hardware may not support larger packets. Check logs:
```
Camera packet size range: 576 - 1500 bytes
Camera adjusted packet size to 1500 bytes (requested 8000)
```

### Problem: Incomplete frames after increasing packet size

**Solution**: 
1. Increase MTU: `sudo ip link set eth0 mtu 9000`
2. Increase timeout: `<param name="gev_block_timeout_ms" value="60000"/>`
3. Check switch supports jumbo frames

## Files Modified

1. **Code**: `/src/galaxy_camera_ros_driver-master/src/galaxy_camera.cpp`
   - Added `gev_packet_size` parameter reading
   - Added automatic range detection
   - Added validation and logging

2. **Launch**: `/src/galaxy_camera_ros_driver-master/launch/MER2-302.launch`
   - Added network parameters section
   - Set default packet size to 8000 bytes
   - Added all GigE Vision parameters

## Next Steps

1. **Compile**: `cd ~/catkin_ws_eagrumo/src/catkin_ws_eas && catkin_make`
2. **Launch**: `roslaunch galaxy_camera MER2-302.launch`
3. **Monitor**: `rostopic hz /galaxy_camera/image_raw`
4. **Adjust**: Modify `gev_packet_size` in launch file as needed

---
Date: 2025-10-07
