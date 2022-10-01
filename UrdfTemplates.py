link = """
    <link name="ref_%(refID)s_link">
      <inertial>
        <mass value="0.010000" />
        <!-- center of mass (com) is defined w.r.t. link local coordinate system -->
        <origin xyz="0.0 0.0 0.0" />
        <inertia  ixx="0.01" ixy="0.0"  ixz="0.0"  iyy="0.01"  iyz="0.0"  izz="0.01" />
      </inertial>
      <visual>
        <!-- visual origin is defined w.r.t. link local coordinate system -->
        <origin xyz="%(pos_x)s %(pos_y)s %(pos_z)s" rpy="%(rot_x)s %(rot_y)s %(rot_z)s" />
        <geometry>
          <mesh filename="%(mesh)s" scale="%(m_scale)s %(m_scale)s %(m_scale)s"/>
        </geometry>
        <!-- color setting is invalid for dae parts (Masahiko)
        <material name="Cyan">
          <color rgba="0 1.0 1.0 1.0"/>
        </material>
        -->
      </visual>
      <collision>
        <!-- collision origin is defined w.r.t. link local coordinate system -->
        <origin xyz="%(bound_x)s %(bound_y)s %(bound_z)s" rpy="%(bound_roll)s %(bound_pitch)s %(bound_yaw)s" />
        <geometry>
          <box size="%(dim_x)s %(dim_y)s %(dim_z)s"/>
        </geometry>
      </collision>
    </link>
"""

joint = """
    <joint name="ref_%(refID)s_joint" type="%(joint_type)s">
      <parent link="%(parent_link)s"/>
      <child link="%(child_link)s"/>
      <origin xyz="%(origin_x)s %(origin_y)s %(origin_z)s" rpy="%(origin_roll)s %(origin_pitch)s %(origin_yaw)s" />
      <axis xyz="%(axis_x)s %(axis_y)s %(axis_z)s" /> %(mimic)s
    </joint>
"""

mimic = """
      <mimic joint="%(joint_name)s" multiplier="%(gear_ratio)s" offset="%(offset)s"/>"""

