#!/usr/bin/env bash
# Durable camera viewer for the laptop. Built on rqt_image_view -- the viewer that
# reliably renders on this laptop. (image_view's compressed image_transport
# subscriber does NOT bind here, even on the vendor stream -> no window; avoid it.)
#
#   ./view.sh          # native eye-in-hand RGB  (/depth_cam/rgb/image_raw)
#   ./view.sh yolo     # jr_vision annotated YOLO stream (/jr/camera/annotated)
#   ./view.sh <topic>  # any base image topic
#
# Transport note (2.4G wifi): raw full-res 30Hz frames fragment and drop to ~1fps,
# so for the *native* camera pick "compressed" in rqt's transport dropdown (rqt
# remembers it next time). The jr_vision annotated stream runs at 4-8Hz and is
# watchable in raw.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

case "${1:-cam}" in
  cam)  TOPIC=/depth_cam/rgb/image_raw ;;
  yolo) TOPIC=/jr/camera/annotated ;;
  *)    TOPIC="$1" ;;
esac

echo "opening rqt_image_view on $TOPIC"
echo "(native 30Hz camera: set the transport dropdown to 'compressed' for smooth/sharp)"
exec ros2 run rqt_image_view rqt_image_view "$TOPIC"
