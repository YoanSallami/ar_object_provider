#!/usr/bin/env python

import time
import sys
import copy
import rospy
from collections import deque
import yaml
import tf2_ros
import re
import math
import argparse
import underworlds
import numpy
from underworlds.helpers import transformations
from underworlds.types import Mesh, MESH
from underworlds.tools.loader import ModelLoader

EPSILON = 0.01
TF_CACHE_TIME = 5.0
MIN_DIST_DETECTION = 0.2


class ARObjectProvider(object):
    def __init__(self, ctx, output_world, ar_objects_file_path, mesh_dir, reference_frame):
        self.ctx = ctx
        self.output = ctx.worlds[output_world]
        self.output_world_name = output_world

        self.mesh_dir = mesh_dir
        self.reference_frame = reference_frame

        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(TF_CACHE_TIME), debug=False)
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.already_created_frame_node_ids = {}

        self.time_table = {}
        self.ar_objects = {}

        self.nodes_transform = {}

        self.ar_object_per_frame = {}
        f = open(ar_objects_file_path, "r")
        self.ar_objects = yaml.load(f)

        first = True
        for ar_frame in self.ar_objects.keys():
            if first:
                self.regex = ar_frame
                first = False
            else:
                self.regex += "|" + ar_frame

    def create_object(self, frame):
        if frame in self.ar_objects:
            node = Mesh(name=self.ar_objects[frame]["name"])
            try:
                nodes_loaded = ModelLoader().load(self.mesh_dir + self.ar_objects[frame]["mesh"], self.ctx,
                                                  world=self.output_world_name, root=None, only_meshes=True,
                                                  scale=self.ar_objects[frame]["scale"])
                for n in nodes_loaded:
                    if n.type == MESH:
                        node.properties["mesh_ids"] = n.properties["mesh_ids"]
                        node.properties["aabb"] = n.properties["aabb"]
                return node
            except Exception as e:
                rospy.logwarn("[env_provider] Exception occurred with %s : %s" % (self.ar_objects[frame]["name"], str(e)))
        return None

    def read_tf_frame(self, frame):
        """
        This method read a /tf frame and create the corresponding node in the world given in constructor
        @param frame:
        @return :
        """
        try:
            msg = self.tfBuffer.lookup_transform(self.reference_frame, frame, rospy.Time(0))
            trans = [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]
            rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
                   msg.transform.rotation.w]

            translation_mat = transformations.translation_matrix(trans)
            rotation_mat = transformations.quaternion_matrix(rot)
            transform = numpy.dot(translation_mat, rotation_mat)

            node = self.create_object(frame)
            node.transformation = transform

            if frame in self.already_created_frame_node_ids:
                node.id = self.already_created_frame_node_ids[frame]
            else:
                self.already_created_frame_node_ids[frame] = node.id
            if node.id in self.nodes_transform:
                if math.sqrt(trans[0] * trans[0] + trans[1] * trans[1] + trans[2] * trans[2]) < MIN_DIST_DETECTION:
                    return None
                if not numpy.allclose(self.nodes_transform[node.id], node.transformation, rtol=0, atol=EPSILON):
                        self.nodes_transform[node.id] = transform
                        return node
            else:
                self.nodes_transform[node.id] = transform
                return node
        except (tf2_ros.TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def update_objects(self):
        nodes_to_update = []
        data = yaml.load(self.tfBuffer.all_frames_as_yaml())
        frames = [p for p in data]
        for frame in frames:
            if re.match(self.regex, frame):
                node = self.read_tf_frame(frame)
                if node:
                    nodes_to_update.append(node)

        if nodes_to_update:
            self.output.scene.nodes.update(nodes_to_update)

    def run(self):
        while not rospy.is_shutdown():
            self.update_objects()


if __name__ == '__main__':
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)
    import argparse

    parser = argparse.ArgumentParser(description="Monitor AR Tag objects")
    parser.add_argument("input_world", help="Underworlds world to listen")
    parser.add_argument("output_world", help="Underworlds world to produce")
    parser.add_argument("ar_objects_file_path", help="The path to localise the AR objects description")
    parser.add_argument("mesh_dir", help="The path to localize the object meshes")
    parser.add_argument("--reference", default="map", help="The reference frame of the system (default map)")
    args = parser.parse_args()

    rospy.init_node("ar_object_provider", anonymous=True)
    with underworlds.Context("AR objects provider") as ctx:
        ARObjectProvider(ctx, args.output_world, args.ar_objects_file_path, args.mesh_dir, args.reference).run()

