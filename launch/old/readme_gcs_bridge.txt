necessary to get messages in mavlink/from

gcs.launch:

<launch>

                <node pkg="mavros" type="gcs_bridge" name="mavlink_bridge">
                        <param name="gcs_url" value="udp://@192.168.2.2:14551" />
                </node>
</launch>

You don't need gcs_bridge anymore. Just call router to add additional link: https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/srv/EndpointAdd.srv

Note that router doesn't pass messages from the same endpoint type. So message coming from GCS will be routed to FCU, but not to other GCS (i.e. QGC).

???


