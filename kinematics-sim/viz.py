def show(q, dh, block=True):
    """Plot PAROL6 at joint configuration q (radians). Reused by FK and IK."""
    dh.plot(q, backend='pyplot', block=block,
            eeframe=True,   # Draw effectors axis triad
            jointaxes=True) # Draw each joint's rotation axis