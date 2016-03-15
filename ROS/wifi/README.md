# ROS WiFi modules

 * wifi : metapackage for all this modules
 * wifi_msgs : messages definition used by these modules
 * wifi_driver : get serial data from GPS and publish NMEA sentences in ROS message
 * wifi_save : publish position-velocity-time fix for ROS

**WiFi driver require the software iwlist. The setuid bit must be set!**
By example :
```bash
chmod +s /sbin/iwlist
```
