# Testing Zero-Copy Architecture

## 🧪 Quick Test Procedure

### 1. Verify Compilation ✅

```bash
cd /home/eagrumo/catkin_ws_eagrumo/src/catkin_ws_eas
catkin_make
```

**Expected**:
```
[100%] Built target galaxy_camera
```

### 2. Launch Camera

```bash
source devel/setup.bash
roslaunch galaxy_camera MER2-302.launch
```

### 3. Monitor Performance (3 Terminals)

#### Terminal 1: Kernel Drop Monitor
```bash
cd /home/eagrumo/catkin_ws_eagrumo/src/catkin_ws_eas
./scripts/monitor_kernel_drops.sh
```

**Expected**:
```
Interface: eth0
Monitoring kernel packet drops (Ctrl+C to stop)
Time       RX Packets   RX Dropped   RX Errors    Status
14:23:01   41234        0            0            ✓ OK
14:23:02   43567        0            0            ✓ OK
```

**Success Criteria**: ✅ Dropped = 0

#### Terminal 2: ROS Logs
```bash
rostopic echo /rosout | grep -E "CALLBACK|PROCESSING|BOTTLENECK"
```

**Expected**:
```
[3] CALLBACK (SDK->Queue): Avg=2.1 ms, Min=1.8 ms, Max=2.5 ms | Queue: 2/4 frames
[3] ✓ SDK callback fast (2.1 ms) - zero-copy pass-through working well

[4] PROCESSING THREAD: Avg=22.3 ms (Conversion=20.1 ms, Publish=2.2 ms)
```

**Success Criteria**: 
- ✅ Callback < 5 ms
- ✅ Queue not overflowing
- ✅ No warnings

#### Terminal 3: Frame Rate Monitor
```bash
rostopic hz /galaxy_camera/image_raw
```

**Expected**:
```
average rate: 5.002
  min: 0.199s max: 0.201s std dev: 0.00050s
```

**Success Criteria**: ✅ Stable ~5 Hz

### 4. Verify Zero Drops with tcpdump

```bash
# Capture 100,000 packets
sudo tcpdump -ni eth0 -c 100000 2>&1 | tail -1
```

**Expected**:
```
100000 packets captured
100000 packets received by filter
0 packets dropped by kernel  ← ✅ THIS IS THE KEY!
```

**Success Criteria**: ✅ 0 packets dropped by kernel

## 📊 Performance Benchmarks

### 4K Camera (4096x3000 @ 5 fps)

| Metric | Target | How to Check |
|--------|--------|--------------|
| SDK Callback | < 5 ms | ROS logs [3] |
| Processing Time | 20-25 ms | ROS logs [4] |
| Queue Depth | 0-2 / 4 | ROS logs [3] |
| Kernel Drops | 0 | monitor_kernel_drops.sh |
| Frame Rate | 5.0 Hz | `rostopic hz` |
| Network RX | ~60 MB/s | `iftop -i eth0` |

### Before vs After Comparison

#### Before (Synchronous)
```bash
# Terminal 1 - tcpdump
sudo tcpdump -ni eth0 -c 100000
# Result: 171,269 packets dropped by kernel (53%)

# Terminal 2 - ROS logs
# [3] CALLBACK: 21-32 ms ← TOO SLOW!

# Terminal 3 - Frame rate
rostopic hz /galaxy_camera/image_raw
# Unstable, frequent gaps
```

#### After (Zero-Copy)
```bash
# Terminal 1 - tcpdump
sudo tcpdump -ni eth0 -c 100000
# Result: 0 packets dropped by kernel ✅

# Terminal 2 - ROS logs
# [3] CALLBACK: 1-3 ms ✅ 10x faster!

# Terminal 3 - Frame rate
rostopic hz /galaxy_camera/image_raw
# Stable 5.002 Hz ✅
```

## 🔍 What to Look For

### ✅ Success Indicators

1. **Fast SDK Callback**
   ```
   [3] CALLBACK (SDK->Queue): Avg=2.1 ms
   [3] ✓ SDK callback fast (2.1 ms)
   ```

2. **Zero Kernel Drops**
   ```
   Interface: eth0
   RX Packets   RX Dropped   RX Errors    Status
   41234        0            0            ✓ OK
   ```

3. **Stable Frame Rate**
   ```
   rostopic hz /galaxy_camera/image_raw
   average rate: 5.002
   ```

4. **Queue Not Overflowing**
   ```
   [3] Queue: 2/4 frames  ← Healthy (not at max)
   ```

### ⚠️ Warning Signs

1. **Slow Callback**
   ```
   [3] ⚠️  SDK callback is slow (8.2 ms)
   ```
   **Action**: Check CPU governor, memory bandwidth

2. **Queue Overflow**
   ```
   [3] ⚠️  QUEUE OVERFLOW! Dropped 10 frames
   ```
   **Action**: Increase `processing_queue_depth`

3. **Kernel Drops**
   ```
   RX Packets   RX Dropped   Status
   41234        523          ⚠ DROPS!
   ```
   **Action**: Run `./scripts/verify_4k_optimization.sh`

4. **Slow Processing**
   ```
   [4] ⚠️  Processing thread is slow (35.2 ms)
   ```
   **Action**: Consider Bayer format or lower resolution

## 🎯 Test Scenarios

### Scenario 1: Normal Operation (5 fps)

**Duration**: 5 minutes

**Expected Results**:
- SDK callback: 1-3 ms
- Processing: 20-25 ms
- Queue depth: 0-2 frames
- Kernel drops: 0
- Frame rate: 5.0 Hz stable

### Scenario 2: Stress Test (High Frame Rate)

**Modify launch file**:
```xml
<param name="acquisition_frame_rate" value="10.0"/>
```

**Expected Results**:
- SDK callback: Still < 5 ms
- Queue depth: 3-4 frames (higher)
- Processing may lag but no drops

### Scenario 3: Recovery Test

**Procedure**:
1. Start camera
2. Pause processing with: `kill -STOP $(pgrep galaxy_camera)`
3. Wait 5 seconds
4. Resume: `kill -CONT $(pgrep galaxy_camera)`

**Expected Results**:
- Queue fills up during pause
- Oldest frames dropped
- Recovers after resume
- No kernel drops

## 📈 Collecting Data for Analysis

### 1. Full Log Capture

```bash
roslaunch galaxy_camera MER2-302.launch 2>&1 | tee camera_test.log
```

### 2. Network Statistics

```bash
# Before starting camera
netstat -i > before_camera.txt

# Run camera for 5 minutes

# After stopping camera
netstat -i > after_camera.txt

# Compare
diff before_camera.txt after_camera.txt
```

### 3. Performance Timeline

```bash
# Monitor all metrics together
watch -n 1 "echo '=== KERNEL ===' && netstat -i | grep eth0 && \
            echo '=== ROS ===' && rostopic hz /galaxy_camera/image_raw && \
            echo '=== CPU ===' && top -p \$(pgrep galaxy_camera) -n 1 -b | tail -5"
```

## 🐛 Common Issues

### Issue: Camera won't start

**Check**:
```bash
# Is camera visible?
GxListDevice

# Is eth0 up?
ip link show eth0

# Are optimizations applied?
./scripts/verify_4k_optimization.sh
```

### Issue: Still seeing kernel drops

**Diagnose**:
```bash
# Check actual RX buffer size
ethtool -g eth0

# Check socket buffer
sysctl net.core.rmem_max net.core.rmem_default

# Verify IRQ affinity
cat /proc/interrupts | grep eth0
```

**Fix**:
```bash
# Reapply optimizations
sudo ./scripts/optimize_for_4k_camera.sh

# Reboot if needed
sudo reboot
```

### Issue: High latency

**Diagnose**:
```bash
# Check queue depth
rostopic echo /rosout | grep "Queue:"
```

**Fix**:
```xml
<!-- Reduce queue depth for lower latency -->
<param name="processing_queue_depth" value="2"/>
```

### Issue: Unstable frame rate

**Diagnose**:
```bash
# Check for processing warnings
rostopic echo /rosout | grep "PROCESSING THREAD"
```

**Fix**:
```xml
<!-- Switch to Bayer format (faster) -->
<param name="pixel_format" value="bayer_rg8"/>
```

## ✅ Success Checklist

- [ ] Compiled successfully (`catkin_make`)
- [ ] Camera launches without errors
- [ ] SDK callback < 5 ms
- [ ] Processing time 20-25 ms
- [ ] Queue not overflowing
- [ ] Kernel drops = 0
- [ ] Frame rate stable at 5 Hz
- [ ] No warnings in ROS logs
- [ ] Can run continuously for 10+ minutes

## 📝 Report Template

```
=== ZERO-COPY ARCHITECTURE TEST REPORT ===

Date: ____________
Camera: MER2-302 (4096x3000 @ 5 fps)

PERFORMANCE METRICS:
- SDK Callback Time: _____ ms (Target: < 5 ms)
- Processing Thread Time: _____ ms (Expected: 20-25 ms)
- Queue Utilization: ___/4 frames
- Kernel Packet Drops: _____ (Target: 0)
- Frame Rate: _____ Hz (Target: 5.0 Hz)

COMPARISON TO BASELINE:
- Before: SDK callback = 21-32 ms
- After: SDK callback = _____ ms
- Improvement: _____x faster

ISSUES ENCOUNTERED:
- None / [Describe issues]

CONCLUSION:
- ✅ Pass / ❌ Fail
- Notes: ________________
```

---

*Happy Testing! 🎉*
