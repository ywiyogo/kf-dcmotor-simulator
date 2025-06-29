# Kalman Filter DC Motor Simulation

A modern C++23 implementation of Kalman filtering for DC motor state estimation with real-time visualization.

## Overview

This application demonstrates the power of Kalman filtering in estimating the state of a dynamic system from noisy measurements. We apply the filter to a DC motor to estimate angular velocity from position measurements, showcasing how mathematical theory translates into practical engineering solutions.

## Problem Statement

### The Challenge
- **What we can measure**: Angular position θ every Δt seconds (with noise)
- **What we want to estimate**: Angular velocity ω (which we cannot directly measure)
- **What we don't know**: The complex physics of the DC motor

### Our Approach
Instead of modeling complex motor physics (torque, current, inductance, etc.), we use a simple kinematic model that captures the essential relationship between position and velocity.

## Mathematical Foundation

### State Vector
Our system state consists of two variables:
```
xₖ = [θ]  ← Angular position (rad)
     [ω]  ← Angular velocity (rad/s)
```

### System Model (Prediction)
We model the system using basic kinematics with unknown acceleration:

```
xₖ = F·xₖ₋₁ + G·αₖ
```

Where:
- **F** = [1  Δt] ← State transition matrix
        [0   1]
- **G** = [½Δt²] ← Acceleration input matrix
        [ Δt ]
- **αₖ** ~ N(0, σₐ²) ← Unknown acceleration (process noise)

This translates to:
- θₖ = θₖ₋₁ + ω·Δt + ½α·Δt² (position update)
- ωₖ = ωₖ₋₁ + α·Δt (velocity update)

### Process Noise
Since acceleration is unknown and random:
```
wₖ ~ N(0, Q)
Q = G·Gᵀ·σₐ²
```

This captures our uncertainty in the model's ability to predict the next state.

### Measurement Model (Update)
We only measure angular position with sensor noise:
```
zₖ = H·xₖ + vₖ
```

Where:
- **H** = [1  0] ← We observe position but not velocity directly
- **vₖ** ~ N(0, R) ← Measurement noise
- **R** = σₜ² ← Measurement noise variance

## Key Features

### 🎯 **State Estimation**
- Real-time estimation of angular velocity from position measurements
- Automatic noise filtering and uncertainty quantification
- Adaptive filtering with performance monitoring

### 📊 **Interactive Visualization**
- Real-time plots of true vs. estimated states
- Error analysis and convergence metrics
- Customizable simulation parameters

### ⚙️ **Modern Implementation**
- C++23 with SIMD-optimized matrix operations
- Cross-platform support (native and WebAssembly)
- Professional UI with docked panels and themes

### 🔧 **Configurable Parameters**
- Process noise standard deviation (σₐ)
- Measurement noise standard deviation (σₜ)
- Sampling time (Δt)
- Various input signal profiles


## Building the Project

### Prerequisites
- CMake 3.28+
- C++23 compatible compiler (GCC 13+, Clang 16+)
- SDL2 development libraries
- Optional: Intel TBB for parallel processing

### Build Commands
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Running the Application
```bash
./bin/KalmanFilterSimulation
```

## Usage

### Interface Layout
- **Left Sidebar**: Parameter tuning and configuration
- **Center Panel**: Real-time plots and visualization
- **Bottom Panel**: Performance statistics and metrics

### Getting Started
1. **Start Simulation**: Click the play button in the menu
2. **Adjust Parameters**: Tune noise levels in the Parameters panel
3. **Monitor Performance**: Watch convergence metrics in Statistics
4. **Export Data**: Save results for further analysis

## Educational Value

### For Students
- Understand Kalman filtering through interactive visualization
- See how mathematical theory applies to real engineering problems
- Experiment with different noise levels and system parameters

### For Engineers
- Template for implementing Kalman filters in C++
- Performance benchmarking and optimization techniques
- Modern software architecture for real-time applications

## Technical Highlights

### Performance Optimizations
- SIMD-vectorized matrix operations
- Parallel processing with Intel TBB
- Memory-efficient data structures
- Real-time capable implementation

### Software Quality
- Modern C++23 features and best practices
- Comprehensive error handling
- Extensive documentation and comments
- Cross-platform compatibility

## Why This Approach Works

The beauty of this kinematic approach lies in its simplicity and robustness:

1. **No Complex Physics**: We don't need to model motor torque, electrical dynamics, or load characteristics
2. **Universal Applicability**: Works with any rotating system, not just DC motors
3. **Automatic Adaptation**: The Kalman filter automatically balances model predictions with sensor measurements
4. **Noise Handling**: Inherently deals with sensor noise and model uncertainties

The Kalman filter gives us the optimal estimate by continuously answering: "Given what I predicted would happen and what I actually measured, what's the best guess of the true state?"
