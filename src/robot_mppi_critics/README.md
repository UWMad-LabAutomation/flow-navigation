# Forward travel preference for FLOW MPPI

`mppi::critics::ForwardTravelCritic` scores predicted body-frame velocities in
the existing MPPI optimizer. It favors forward travel on clear paths while
keeping the Omni model available for obstacle avoidance.

Outside a 15-degree forward cone, each predicted velocity contributes
`max(abs(vy) - tan(half_angle) * max(vx, 0), 0)` to the cost. This permits small
tracking corrections. The default weight is 60. It drops to 4 when both:

- the robot faces the local route within 0.5 rad; and
- a known lethal obstacle cell lies 0.3955-1.5 m ahead, within +/-0.321 m laterally.

Unknown cells do not trigger the reduced penalty. Checks use the existing local
costmap (static map plus camera observations). MPPI footprint collision costs,
the total-speed limiter and collision monitor remain in the velocity chain.
The camera's side/rear blind spots still require care and hardware validation.

This plugin changes trajectory scoring; it does not clamp sideways commands
after planning. The configuration combines it with forward Dubins Hybrid A*
paths, PathAngle/PreferForward critics and a lower Twirling penalty so the robot
can turn toward its route. Slow reverse is limited to 0.10 m/s.

`ForwardTravelCritic` is registered under the stock Jazzy MPPI critic base class.
Build the package before launching navigation. Unit tests cover the forward
cone and costmap obstacle/unknown/heading checks; live validation exercises
clear-path startup and inserted-obstacle navigation.
