"""Offline training of each joint's port-Hamiltonian model of itself (B11).

The joint's commissioning log (NX-13) is all a model learns from. The trained model is
frozen, exported as versioned weights, and run inside the joint by the Rust node (NX-15).
Python never sits in the control loop.
"""
