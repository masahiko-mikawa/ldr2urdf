# ldr2urdf
Conversion tool from LDraw to URDF for ROS2

How to use:
    1. Make a 3D LEGO CAD model in LDraw format using LeoCAD
        If you do not know LDraw or LeoCAD, please see the following links.
            https://www.ldraw.org
            https://www.leocad.org
           
    2. Input the following command
        python3 ldr2urdf.py hoge.ldr
  
    3. Then hoge.urdf is made.
  
Limitations:
    - Joints and links cannot be defined by LeoCAD, so we must use LeoCAD's functions of grouping and comments.
      Please see the ldraw example file.
