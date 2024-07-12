# AUV_simulator
Numerically solve the dynamics and control of a simplified AUV (Autonomous Underwater Vehicle) model.

A torphedo-shaped AUV is simulated to follow a sequence of target locations and to avoid an obstacle within a 150 m^2 box. The AUV is subjected to a uniform background marine current. The AUV has a single maneuvering surface, which provides a trajectory control via a virtual PID controller. The equations of motion are numerically integrated by a fourth-order accurate Runge-Kutta scheme. The hydrodynamic and propulsion models are taken from the following research papers:

- Jorgensen, L. H. Prediction of static aerodynamic characteristics for spaceshuttle-like and other bodies at angles of attack from 0 deg to 180 deg. Tech. Rep. (1973).
- Yuh, J. Modeling and control of underwater robotic vehicles. IEEE Transactions on Systems, man, and Cybernetics 20.6 (1990): 1475-1483.
