# Full-size arm and Astra Pro tray prototype

[arm_mast_astra_tray_full_size.stl](arm_mast_astra_tray_full_size.stl) is a
one-piece model in **millimetres**. It contains:

- A 420 x 480 x 4 mm tray with a raised perimeter.
- A locating ring for the existing robot `base_link.STL`, measured at
  60.63 mm maximum radius. The ring has 1.5 mm nominal radial clearance.
- A braced, hollow mast at the URDF mount position (X +220, Y -300 mm).
- An open-front U cradle for a 165 x 30 x 40 mm Astra Pro camera. The camera
  optical origin corresponds to (X +220, Y -290, Z +500 mm), pitched 50°
  downward. The camera envelope was checked for clearance in the cradle.

The STL's bounding box is **420 x 480 x 545 mm**. It needs a printer with a
build volume larger than that, including a little margin for brim and travel.
Print with the tray floor on the bed, a wide brim, and supports under the
camera cradle; check the mast and cradle strength in the slicer preview.
Do not scale the model to fit a smaller printer: the arm ring and camera
cradle would no longer fit. For a common 220 mm printer, this design needs
segmentation and mechanical joints; this one-piece STL is the master geometry.

This is a first-fit prototype. The arm ring locates the arm but does not clamp
it, because mounting screw locations are not specified by the robot mesh.
The cradle shape follows the camera's published outside dimensions, but
measure the actual camera and its cable/connector before a long print.
Add a retaining strap and mechanical fasteners before operating the robot.
The camera's published minimum depth range is about 0.6 m; confirm the real
work area lies within its usable range.

Regenerate the STL from the URDF's camera mount defaults and robot base mesh:

```bash
python3 cad/tray/generate_tray.py
```

The script uses NumPy and VTK. It checks for open or non-manifold edges and
reports surface connectivity. The camera-body fit check used sample points
inside a 28 x 160 x 38 mm box centered on the nominal camera envelope.

## View the assembled robot in RViz

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_description tray_rviz.launch.py
```

This starts the arm, printed tray, and Astra Pro at the cradle's design pose.
The installed mesh is copied to
`src/robot_arm_description/meshes/arm_mast_astra_tray.stl` and scaled by 0.001
in the URDF, because the STL uses millimetres and ROS uses metres. After
regenerating the printable STL, update this mesh copy and rebuild the description
package. The tray is display geometry; collision checking for the printed
fixture has not been configured.

The Gazebo launch also enables `use_tray:=true` by default. Use
`use_tray:=false` to display the original standalone camera mast. The printed
cradle has a fixed pose; changing the camera arguments requires regenerating
the tray to maintain the fit.
