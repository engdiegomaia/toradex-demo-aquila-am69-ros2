# F1 — evidência de execução (learn mode containerizado)

Host x86, 2026-08-14 19:11 UTC. Kernel: 7.0.0-28-generic
Docker 29.7.2, Compose 5.0.2

## Serviços
nav  running
perception  running
sim  running
tools  running
viz  running

## Contrato de tópicos
/clock
/demo/camera/camera_info
/demo/camera/depth_image
/demo/camera/image_raw
/demo/camera/points
/demo/cmd_vel
/demo/imu
/demo/odom
/demo/perception/detection_cloud
/demo/perception/detections
/demo/scan

## Taxas
/clock                             average rate: 334.385
/demo/odom                         average rate: 27.752
/demo/scan                         average rate: 9.986
/demo/camera/image_raw             average rate: 9.963
/demo/perception/detections        average rate: 10.004

## Nav2 lifecycle
amcl                 label='active'
bt_navigator         label='active'
controller_server    label='active'
planner_server       label='active'

## Goal Nav2

Result:
    error_code: 0
error_msg: ''

Goal finished with status: SUCCEEDED
