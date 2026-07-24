# Vision configuration (v1.3+)
#
# Algorithm-agnostic camera calibration and method-specific tunables live here.
#
# Bundles:
#   hsv_blob_overhead.yml — synthetic orange disk fixtures (640×480)
#   hsv_blob_tennis_yellow.yml — yellow-green tennis / fetch balls (web demo)
#   omx_portal_overhead.yml — OM-X MuJoCo portal Cam-A (1280×720)
#   omy_portal_overhead.yml — OMY MuJoCo portal Cam-A (1280×720)
#   demo_web_balls.yml — Wikimedia URL list for scripts/vision_web_ball_gallery.py
#
# MuJoCo portal benchmark:
#   MUJOCO_GL=egl PYOPENGL_PLATFORM=egl python3 scripts/benchmark_mujoco_vision.py
#
# Gallery:
#   python3 scripts/vision_web_ball_gallery.py
#   python3 scripts/vision_web_ball_gallery.py --upload-r2
#
# See docs/vision/camera-layout.md and docs/modules/vision.md.
