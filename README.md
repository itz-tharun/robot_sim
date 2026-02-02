# Robot Motion Simulator

A simple 2D robot motion simulator implementing a unicycle model using
discrete-time integration.

## Features
- Velocity-based motion model
- Time-step (dt) integration
- Console-based simulation

## Motion Model
x += v * cos(theta) * dt  
y += v * sin(theta) * dt  
theta += omega * dt  

## Purpose
Built to understand:
- Motion models
- Time discretization
- Error accumulation
- Foundations of localization

## Run
```bash
python motion_sim.py
