# XIMEA Driver

Install XIMEA SDK:
```
./tools/install_sdk.sh
```

Build driver:
```
./build.sh
```

Run driver:
```
ros2 launch ximea_driver ximea_driver.launch.py
```

## USB Buffer Size

If you encounter error code 13 during acquisition, you may need to increase
the USB buffer size. You can do this with the following command:
```
sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null <<<0
```

You can run this automatically by adding it to `/etc/rc.local`.
