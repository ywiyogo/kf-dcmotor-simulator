# Kalman Filter - A Simple Introduction

The Kalman filter is understandable. Let's understand it step by step.

## What is a Kalman Filter?

The Kalman filter is a mathematical algorithm that helps us estimate the true state of a system when we can only make noisy, imperfect measurements. Think of it as a smart way to combine:
- What we predict should happen (based on physics/models)
- What we actually observe (through sensors)

The filter continuously updates its estimate by weighing these two sources of information.

## The Mathematical Model

The Kalman filter works with linear systems that can be described by two equations:

### System Dynamics (Prediction Step)
```
xₖ = F·xₖ₋₁ + B·uₖ + wₖ
```

This equation describes how the system evolves over time:
- `xₖ`: Current state vector (what we want to know)
- `F`: State transition matrix (how states change naturally)
- `xₖ₋₁`: Previous state
- `B`: Control input matrix
- `uₖ`: Control input (if any)
- `wₖ`: Process noise (uncertainty in our model)

### Measurement Model (Update Step)
```
zₖ = H·xₖ + vₖ
```

This equation relates what we can measure to the actual state:
- `zₖ`: Measurement vector (what sensors tell us)
- `H`: Measurement matrix (how states relate to measurements)
- `vₖ`: Measurement noise (sensor uncertainty)

## Application to DC Motor Position and Velocity Estimation

### The Problem
We have a DC motor and we want to:
- **Measure**: Angular position θ every Δt seconds
- **Estimate**: Angular velocity ω (which we cannot directly measure)

### Our State Vector
```
xₖ = [θ]  ← Angular position
     [ω]  ← Angular velocity
```

### System Model (Kinematics)
We use simple kinematics since we don't assume knowledge of motor physics:

```
Position: θₖ = θₖ₋₁ + ω·Δt
Velocity: ωₖ = ωₖ₋₁ + α·Δt
```

Where α is unknown acceleration (treated as noise).

### State Transition Matrix F
```
F = [1  Δt]  ← Position depends on previous position + velocity×time
    [0   1]  ← Velocity stays constant (no known acceleration)
```

### Process Noise
We assume acceleration α is random with standard deviation σₐ:

```
G = [½Δt²]  ← How acceleration affects position
    [ Δt ]  ← How acceleration affects velocity

Q = G·Gᵀ·σₐ²  ← Process noise covariance matrix
```

### Measurement Model
We only measure angular position:

```
H = [1  0]  ← We observe position but not velocity directly

R = σₜ²     ← Measurement noise variance (sensor accuracy)
```

## How It Works

1. **Predict**: Use the model to predict where the system should be
2. **Update**: When a measurement arrives, adjust the prediction
3. **Repeat**: Continue this predict-update cycle

The filter automatically balances:
- **Model confidence**: How much we trust our prediction
- **Sensor confidence**: How much we trust our measurement

If the model is accurate, it relies more on predictions. If sensors are good, it relies more on measurements.

## Why This Approach?

This kinematic approach is robust because:
- No need to know complex motor physics
- Works with any rotating system
- Simple to implement and understand
- Automatically handles sensor noise and model uncertainties

The Kalman filter gives us the best possible estimate of both position and velocity, even though we only measure position!