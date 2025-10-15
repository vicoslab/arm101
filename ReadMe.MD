# ViCoS SO-ARM101 Resources

First install and source [lerobot](https://huggingface.co/docs/lerobot/en/installation).

```bash
sudo apt-get install cmake build-essential python-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev pkg-config virtualenv

virtualenv .
source bin/activate

pip install lerobot
pip install 'lerobot[feetech]'
```

## UDEV Setup

Copy `99-custom.rules` from the `udev_rules` dir to `/etc/udev/rules.d` or run:

```bash
sudo cp udev_rules/99-custom.rules /etc/udev/rules.d/99-custom.rules
```

Then update udev:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

When arms are plugged in they should now map to `/dev/arm_XY`, e.g.

- arm F2 -> `/dev/arm_f2` 
 
- arm L4 -> `/dev/arm_l4`.

## Lerobot Calibrations

Copy the existing calibration files from `calibrations` to lerobot's cache directory at `~/.cache/huggingface/lerobot/calibration/robots/so101_follower`

Or run:
```bash
cp calibrations/* ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/
```

Calibrations were created by plugging in both arms and running e.g.:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/arm_f7 --robot.id=arm_f7

lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/arm_l7 --robot.id=arm_l7
```

## Run leader-follower teleop

Replace the arm IDs with your specific leader and follower arm and run:
```bash
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/arm_f7 \
    --robot.id=arm_f7 \
    --teleop.type=so101_leader \
    --teleop.port=/dev/arm_l7 \
    --teleop.id=arm_l7
```


# Run keyboard teleop

Only plug in the follower robot, run `python3 teleop.py`, recalibrate and the robot should set itself into its default position.