# DC Motor Model - Understanding the Physics

A DC motor is an electrical machine that converts electrical energy into mechanical rotation. Let's break down how it works and the mathematics behind it.

## Physical Components

### Electrical Side
- **Armature Circuit**: The rotating coil with:
  - `R`: Electrical resistance (Ohms)
  - `L`: Electrical inductance (Henries)
- **Applied Voltage**: `V` (Volts)
- **Current**: `i` (Amperes)

### Mechanical Side
- **Rotor**: The rotating part with:
  - `J`: Moment of inertia (kg·m²) - resistance to rotational acceleration
  - `b`: Damping coefficient (N·m·s/rad) - friction and air resistance
- **Angular Velocity**: `ω` (rad/s)
- **Torque**: `τ` (N·m)

### Coupling Between Electrical and Mechanical
- **Torque Constant**: `kt` (N·m/A) - converts current to torque
- **Back-EMF Constant**: `ke` (V·s/rad) - converts rotation to voltage

## How It Works

### 1. Electrical Equation
When current flows through the armature, it encounters:
- Resistance that opposes current flow
- Inductance that opposes current changes
- Back-EMF that opposes the applied voltage

```
L·(di/dt) + R·i = V - ke·ω
```

**In plain English**: The applied voltage drives current through resistance and inductance, but the spinning motor generates a back-voltage that opposes this.

### 2. Mechanical Equation
The electrical current creates torque that must overcome:
- Inertia (resistance to acceleration)
- Damping (friction and air resistance)

```
J·(dω/dt) + b·ω = kt·i
```

**In plain English**: The torque from current spins the motor against inertia and friction.

### 3. State-Space Form
For simulation, we solve for the highest derivatives:

```
dω/dt = (kt·i - b·ω) / J     ← Angular acceleration
di/dt = (V - ke·ω - R·i) / L  ← Current change rate
```

## Real Motor Parameters

We use parameters from the Moog C23-L33-W10 DC motor:

### Electrical Properties
- **Terminal Resistance (R)**: 0.6 Ω
  - *How much the windings resist current flow*
- **Terminal Inductance (L)**: 0.035 H (35 mH)
  - *How much the windings resist current changes*

### Electromechanical Properties
- **Torque Sensitivity (kt)**: 0.0187 N·m/A
  - *How much torque per unit of current*
- **Back-EMF Constant (ke)**: 0.0191 V/(rad/s)
  - *How much voltage generated per unit of rotation speed*

### Mechanical Properties
- **Damping Factor (b)**: 0.0000095 N·m/(rad/s)
  - *Friction and air resistance*
- **Rotational Inertia (J)**: 0.000125 kg·m²
  - *For a 0.1 kg disc with 5 cm radius: J = ½mr² = 0.5 × 0.1 × (0.05)²*

## Why These Equations Matter

### For Control Systems
- The electrical time constant `τₑ = L/R ≈ 0.058s` shows how quickly current responds
- The mechanical time constant `τₘ = J/b ≈ 13.2s` shows how quickly speed responds
- Since `τₘ >> τₑ`, current changes much faster than speed

### For Kalman Filter Design
- These equations help us understand what we can measure vs. what we want to estimate
- They define the relationship between applied voltage (input) and position/velocity (states)
- The coupling between electrical and mechanical sides creates the dynamics we need to model

## Practical Insights

1. **Current Control**: Since electrical dynamics are fast, we can treat current as almost instantaneous compared to mechanical motion.

2. **Back-EMF Effect**: As the motor spins faster, it generates more back-EMF, naturally limiting its speed for a given voltage.

3. **Load Impact**: The inertia `J` dominates the mechanical response time. A heavier load makes the motor respond more slowly.

4. **Efficiency**: The ratio `kt/ke` affects motor efficiency. For this motor, they're nearly equal (0.0187 vs 0.0191), which is typical for well-designed motors.

This model forms the foundation for understanding how voltage inputs create position and velocity outputs, which is exactly what our Kalman filter needs to estimate!