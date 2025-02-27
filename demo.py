# Copyright (c) Facebook, Inc. and its affiliates.
# 
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

""" Demo of using VoteNet 3D object detector to detect objects from a point cloud.
"""

import os
import sys
import numpy as np
import argparse
import importlib
import time

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', default='sunrgbd', help='Dataset: sunrgbd or scannet [default: sunrgbd]')
    parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
    parser.add_argument('--pc_path', type=str, help='Input point cloud path')
    FLAGS = parser.parse_args()
    return vars(FLAGS)

import torch
import torch.nn as nn
import torch.optim as optim

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = BASE_DIR
sys.path.append(os.path.join(ROOT_DIR, 'utils'))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
from pc_util import random_sampling, read_ply, write_ply
from ap_helper import parse_predictions

def preprocess_point_cloud(point_cloud, num_point=20000):
    ''' Prepare the numpy point cloud (N,3) for forward pass '''
    point_cloud = point_cloud[:,0:3] # do not use color for now
    floor_height = np.percentile(point_cloud[:,2],0.99)
    height = point_cloud[:,2] - floor_height
    point_cloud = np.concatenate([point_cloud, np.expand_dims(height, 1)],1) # (N,4) or (N,7)
    point_cloud = random_sampling(point_cloud, num_point)
    pc = np.expand_dims(point_cloud.astype(np.float32), 0) # (1,40000,4)
    return pc


def point_cloud_to_detections(point_cloud=None, pc_path=None,
                              dataset='sunrgbd', num_point=20000):
    if point_cloud is not None and pc_path is not None:
        write_ply(point_cloud, pc_path)
    if point_cloud is not None and pc_path is None:
        raise ValueError("Please provide both pc_path and point_cloud. Not just one")

    # Set file paths and dataset config
    demo_dir = os.path.join(BASE_DIR, 'demo_files') 
    if dataset == 'sunrgbd':
        sys.path.append(os.path.join(ROOT_DIR, 'sunrgbd'))
        from sunrgbd_detection_dataset import DC # dataset config
        checkpoint_path = os.path.join(demo_dir, 'pretrained_votenet_on_sunrgbd.tar')
        pc_path = pc_path or os.path.join(demo_dir, 'input_pc_sunrgbd.ply')
    elif dataset == 'scannet':
        sys.path.append(os.path.join(ROOT_DIR, 'scannet'))
        from scannet_detection_dataset import DC # dataset config
        checkpoint_path = os.path.join(demo_dir, 'pretrained_votenet_on_scannet.tar')
        pc_path = pc_path or os.path.join(demo_dir, 'input_pc_scannet.ply')
    else:
        print('Unkown dataset %s. Exiting.'%(dataset))
        exit(-1)


    eval_config_dict = {'remove_empty_box': True, 'use_3d_nms': True, 'nms_iou': 0.25,
        'use_old_type_nms': False, 'cls_nms': False, 'per_class_proposal': False,
        'conf_thresh': 0.5, 'dataset_config': DC}

    # Init the model and optimzier
    MODEL = importlib.import_module('votenet') # import network module
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net = MODEL.VoteNet(num_proposal=256, input_feature_dim=1, vote_factor=1,
        sampling='seed_fps', num_class=DC.num_class,
        num_heading_bin=DC.num_heading_bin,
        num_size_cluster=DC.num_size_cluster,
        mean_size_arr=DC.mean_size_arr).to(device)
    print('Constructed model.')
    
    # Load checkpoint
    optimizer = optim.Adam(net.parameters(), lr=0.001)
    checkpoint = torch.load(checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    epoch = checkpoint['epoch']
    print("Loaded checkpoint %s (epoch: %d)"%(checkpoint_path, epoch))
   
    # Load and preprocess input point cloud 
    net.eval() # set model to eval mode (for bn and dp)
    print("Loading point cloud from {}".format(pc_path))
    point_cloud = read_ply(pc_path)
    pc = preprocess_point_cloud(point_cloud, num_point=num_point)
    print('Loaded point cloud data: %s'%(pc_path))
   
    # Model inference
    inputs = {'point_clouds': torch.from_numpy(pc).to(device)}
    tic = time.time()
    end_points = net(inputs)
    toc = time.time()
    print('Inference time: %f'%(toc-tic))
    end_points['point_clouds'] = inputs['point_clouds']
    pred_map_cls = parse_predictions(end_points, eval_config_dict)
    print('Finished detection. %d object detected.'%(len(pred_map_cls[0])))
    if not len(pred_map_cls[0]):
        return
    for row in pred_map_cls:
        for cls in row:
            print('pred_map_cls: ', cls[0], '; conf:', cls[2])

    pc_path_prefix, _ = os.path.splitext(pc_path)
    dump_dir = "{pc_path_prefix}_results".format(pc_path_prefix=pc_path_prefix)
    if not os.path.exists(dump_dir): os.mkdir(dump_dir) 
    MODEL.dump_results(end_points, dump_dir, DC, True)
    print('Dumped detection results to folder %s'%(dump_dir))
    return read_ply(os.path.join(dump_dir, "000000_pred_confident_bbox.ply")), pred_map_cls

if __name__== '__main__':
    point_cloud_to_detections(**parse_args())
