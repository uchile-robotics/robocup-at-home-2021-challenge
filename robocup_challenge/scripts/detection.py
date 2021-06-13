# -*- coding: utf-8 -*-
#!/usr/bin/env python
import tf
import tf2_ros
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
from yolov5.detect import YoloV5
import cv2
from geometry_msgs.msg import TransformStamped

class RGBD():

    def __init__(self):
        self._br = tf.TransformBroadcaster()
        self.model = YoloV5(weights='ycb_v2.pt')
        self.names = self.model.names
        #self._cloud_sub = rospy.Subscriber(
        #    "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
        #    PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        self.xyz = []
        self.labels = []    
        self._w_image = 640
        self._h_image = 360    


    def detect(self, sort=True):
        """Funcion que retorna 2 listas, una con los xyz de la nube de puntos (ordenadas del más cercano al más lejano si 
        sort = True y una lista con sus labels respectivos.
        """
        msg = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2)
        self._points_data = ros_numpy.numpify(msg)
        self._image_data = self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
        self.xyz = []
        self.labels = []
        _xy = []
        if not self._image_data is None:    
            detections = self.model.detect(self._image_data)
            if not detections == []:
                for i, (x1, y1, x2, y2, cls_conf, i_label) in enumerate(detections):
                    _x = int(round(x1.item()))
                    _y = int(round(y1.item()))
                    _w = int(round(x2.item() - x1.item()))
                    _h = int(round(y2.item() - y1.item()))
                    conf = cls_conf
                    label = self.names[int(i_label.item())]
                    _xy.append((_x,_y,label))

                if sort:
                    _xy = self.sort_objects(_xy) # ordenar los objetos si es necesario
                    
                for i, (_x, _y, label) in enumerate(_xy):    # transformación coordenada de imagen a nube de puntos
                    x = self._points_data['x'][_y,_x]
                    y = self._points_data['y'][_y,_x]
                    z = self._points_data['z'][_y,_x]
                    if not np.isnan(z):
                        self.xyz.append([x,y,z]) # agregar a self._xyz las coordenadas con respecto a la nube
                        self.labels.append(label)  
                

            return self.xyz, self.labels    

        else:
            return None, None 

    def sort_objects(self, xy):
        x_ref, y_ref = self._w_image/2, self._h_image
        xy = sorted(xy, key=lambda x: (x[0]-x_ref)**2 + (x[1]-y_ref)**2) # ordenar puntos
        return xy       

    def get_image(self):
        return self._image_data

    def get_points(self):
        return self._points_data

    def get_xyz(self):
        return self.xyz

    def get_labels(self):
        return self.labels

    def get_relative_coordinate(self, parent, child):

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        print(tfBuffer)
        trans = TransformStamped()
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform(parent, child,
                                                rospy.Time().now(),
                                                rospy.Duration(4.0))
                break
            except (tf2_ros.ExtrapolationException):
                pass

        return trans.transform    