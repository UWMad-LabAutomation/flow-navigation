# robot_description

This package contains the initial simplified URDF/Xacro description of the
I2RT LinearBot FLOW base.

The model currently provides the navigation frames `base_footprint` and
`base_link`, along with a box visual, collision geometry, and inertial data.
The documented 0.751 m by 0.542 m footprint and approximately 80 kg mass are
used. The base-only height is not documented yet, so the model uses a
provisional 0.200 m height. The center of mass is also undetermined and is
provisionally placed at the geometric center of that box.

The inertia is calculated for a uniform rectangular solid. Replace the
provisional height, center of mass, and inertia with measured values before
using this model for dynamic simulation or hardware control.
