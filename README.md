# Autonomy basics
Simulations going through basic localization and control techniques behind AI in robotics.

## Localization
Histogram filter inferring an agent's location after movement using environment readings and Bayesian probability.
  
*TODO: Rewrite in C++, maybe make it a 3D space or add multiple sensors

## PID control
Control steering in a car going around a racetrack, keeping it on track through noise in motion commands and constant drift in steering angle.

## Search and Control
Put all the parts together and use A* search to find the optimal path through a randomly generated environment, creating a smooth path for a car to travel around obstacles to a goal position.
Car steering is directed using a PID controller with noisy inputs and car position in the environment is localized through a particle filter with uncertain measurements.
