# -*- coding: utf-8 -*-
#!/usr/bin/env python
import os
import sys

import tf
import cv2
import tf2_ros
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped

from yolov5.detect_scoring import YoloV5

#np.set_printoptions(threshold=sys.maxsize)

class RGBD():
    u"""RGB-Dデータを扱うクラス"""

    def __init__(self):
        self._br = tf.TransformBroadcaster()
        # ポイントクラウドのサブスクライバのコールバックに_cloud_cbメソッドを登録
        self.model = YoloV5(weights='ycb_v2.pt')
        self.names = self.model.names
        self.bridge = CvBridge()
        # self._cloud_sub = rospy.Subscriber(
        #    "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
        #    PointCloud2, self._cloud_cb)
        self.pcloud_pub = rospy.Publisher("/hsrb/head_rgbd_sensor/depth_registered/rectified_points_mask", PointCloud2, queue_size=2)
        self._points_data = None
        self._image_data = None
        self.xyz = []
        self.labels = []
        self._w_image = 640
        self._h_image = 480

    
    def detect(self, sort=True, save=False, segment=True):
        """Funcion que retorna 2 listas, una con los xyz de la nube de puntos (ordenadas del más cercano al más lejano si 
        sort = True y una lista con sus labels respectivos.
        """
        msg = rospy.wait_for_message(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2)
        self._points_data = ros_numpy.numpify(msg)
        self._image_data = self._points_data['rgb'].view(
            (np.uint8, 4))[..., [0, 1, 2]]
        self.xyz = []
        self.labels = []
        _xy = []
        if not self._image_data is None:
            detections = self.model.detect(self._image_data)
            if not detections == []:
                for i, (x1, y1, x2, y2, cls_conf, i_label) in enumerate(detections):
                    _x = int(round(x2.item() - x1.item())/2 + x1.item())
                    _y = int(round(y2.item() - y1.item())/2 + y1.item())
                    _w = int(round(x2.item() - x1.item()))
                    _h = int(round(y2.item() - y1.item()))
                    conf = cls_conf
                    label = self.names[int(i_label.item())]
                    _xy.append((_x, _y, label))

                if sort:
                    # ordenar los objetos si es necesario
                    _xy = self.sort_objects(_xy)

                # transformación coordenada de imagen a nube de puntos
                for i, (_x, _y, label) in enumerate(_xy):
                    x = self._points_data['x'][_y, _x]
                    y = self._points_data['y'][_y, _x]
                    z = self._points_data['z'][_y, _x]
                    if not np.isnan(z):
                        # agregar a self._xyz las coordenadas con respecto a la nube
                        self.xyz.append([x, y, z])
                        self.labels.append(label)
                

            return self.xyz, self.labels

        else:
            return None, None

    def segmentation(self, sort=True):
        """Función que retorna la máscara del objeto más cercano.
        Args:
            sort (bool, optional): [Si se desea ordenar del más cercano
                                    al más lejano]. Defaults to True.
        Returns:
            [numpy array]: [Máscara de objeto más cercano al robot]
        """
        msg = rospy.wait_for_message(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2)
        self._points_data = ros_numpy.numpify(msg)
        self._image_data = self._points_data['rgb'].view(
            (np.uint8, 4))[..., [0, 1, 2]]
        _xywh = []
        if not self._image_data is None:
            detections = self.model.detect(self._image_data)
            if not detections == []:
                for i, (x1, y1, x2, y2, cls_conf, i_label) in enumerate(detections):
                    _x = int(round(x2.item() - x1.item())/2 + x1.item())
                    _y = int(round(y2.item() - y1.item())/2 + y1.item())
                    _w = int(round(x2.item() - x1.item()))
                    _h = int(round(y2.item() - y1.item()))
                    conf = cls_conf
                    label = self.names[int(i_label.item())]
                    _xywh.append((_x, _y, _w, _h, label))

                if sort:
                    # ordenar los objetos si es necesario
                    _xywh = self.sort_objects(_xywh)

                obj = _xywh[0]  # object nearest
                mask_obj = self.segmentation_object(obj)
                return mask_obj
            return None
        return None

    def segmentation_object(self, obj):
        _point_data = self._points_data.copy()
        frame_rgb = self._image_data.copy()
        frame_mask = np.zeros((480, 640))
        x, y, w, h, label = obj
        frame_obj = frame_rgb[int(y-h/2):int(y+h/2), int(x-w/2):int(x+w/2)]

        frame_hsv = cv2.cvtColor(
            frame_obj, cv2.COLOR_RGB2HSV_FULL)  # convert to hsv
        # valores mesa RGB
        lower_rgb = np.array([234, 229, 234])
        upper_rgb = np.array([255, 255, 255])
        mask_mesa = 255-cv2.inRange(frame_obj, lower_rgb, upper_rgb)

        # valores para piso HSV
        lower_hsv = np.array([144, 109, 110])
        upper_hsv = np.array([150, 255, 255])
        mask_piso = 255-cv2.inRange(frame_hsv, lower_hsv, upper_hsv)
        kernel = np.ones((3, 3), np.uint8)
        mask_piso = cv2.erode(mask_piso, kernel)

        result = cv2.bitwise_and(mask_piso, mask_mesa)
        frame_mask[int(y-h/2):int(y+h/2), int(x-w/2):int(x+w/2)] = result
        # cv2.imshow('result', frame_mask)
        # cv2.imshow('frame', frame_rgb)
        # cv2.waitKey(1)
        result_x = (self._points_data['x']*frame_mask/255)
        result_y = (self._points_data['y']*frame_mask/255)
        result_z = (self._points_data['z']*frame_mask/255)

        _point_data['x'] = result_x
        _point_data['y'] = result_y
        _point_data['z'] = result_z

        msg = ros_numpy.msgify(PointCloud2, _point_data)
        msg.header.frame_id = "/head_rgbd_sensor_rgb_frame"
        
        self.pcloud_pub.publish(msg)
        return frame_mask

    def sort_objects(self, xy):
        x_ref, y_ref = self._w_image/2, self._h_image
        # ordenar puntos por distancia
        xy = sorted(xy, key=lambda x: (x[0]-x_ref)**2 + (x[1]-y_ref)**2)
        return xy

    def get_image(self):
        u"""画像を取得する関数"""
        return self._image_data

    def get_points(self):
        u"""ポイントクラウドを取得する関数"""
        return self._points_data

    def get_xyz(self):
        u"""抽出領域の画像を取得する関数"""
        return self.xyz

    def get_labels(self):
        return self.labels

    def get_relative_coordinate(self, parent, child):
        u"""相対座標を取得する関数

        引数：
            parent (str): 親の座標系
            child (str): 子の座標系

        """

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        print(tfBuffer)
        trans = TransformStamped()
        while not rospy.is_shutdown():
            try:
                # 4秒待機して各tfが存在すれば相対関係をセット
                trans = tfBuffer.lookup_transform(parent, child,
                                                  rospy.Time().now(),
                                                  rospy.Duration(4.0))
                break
            except (tf2_ros.ExtrapolationException):
                pass

        return trans.transform

    def _cloud_cb(self, msg):
        self._points_data = ros_numpy.numpify(msg)
        self._image_data = self._points_data['rgb'].view(
            (np.uint8, 4))[..., [2, 1, 0]]
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
                    x = self._points_data['x'][_y, _x]
                    y = self._points_data['y'][_y, _x]
                    z = self._points_data['z'][_y, _x]
                    if not np.isnan(z):
                        self._xyz.append([x, y, z])
                        self._frame_name.append(label)
                        # print('{},{},{},{}'.format(x,y,z,label))

            if i in range(len(self._frame_name)):
                (x, y, z) = self._xyz[i]
                self._br.sendTransform(
                    (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
                    self._frame_name[i],
                    msg.header.frame_id)

        else:
            return


#rospy.init_node('detector_xyz')
#m = RGBD()
#while not rospy.is_shutdown():
#    m.segmentation()
