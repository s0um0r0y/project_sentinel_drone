#!/usr/bin/env python3
from qgis.utils import iface
import rospy
from sentinel_drone.msg import Geolocation
from geometry_msgs.msg import geometry_msgs
from std_msgs.msg import String
canvas=iface.mapCanvas()
layer = iface.activeLayer()
dir(layer)
def geolocation_pub():
    rospy.init_node('geolocation_node',anonymous=True)
    pub=rospy.Publisher('Geolocation',Geolocation,queue_size=10)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        #put the qgis code here(for publisher)
        for f in layer.getFeatures():
            print(f['name'], f['iata_code'])
            for f in layer.getFeatures():
                geom=f.geometry_msgs()
                print(geom.asPoint())
            for f in layer.getFeatures():
                geom=f.geometry_msgs()
                print(geom.asPoint().x())
            for f in layer.getFeatures():
                geom=f.geometry_msgs()
                print('%s,%s,%f,%f' %(f['name'],f['iata_code'],geom.asPoint().y(), geom.asPoint().x())
        pub.publish(Geolocation)
        rate.sleep()
output_file = open('/home/soumoroy/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/subscriber.py', 'w')
for f in layer.getFeatures():
  geom = f.geometry()
  line = '%s, %s, %f, %f\n' % (f['name'], f['iata_code'],
          geom.asPoint().y(), geom.asPoint().x())
  unicode_line = line.encode('utf-8')
  output_file.write(unicode_line)
output_file.close()

if __name__=="__main__":
    try:
        geolocation_pub()
    except  rospy.ROSInterruptException:
        pass