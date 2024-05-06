#!/usr/bin/env python3
import os
import torch
import torch.nn as nn
import sys
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import time
repo_path, _ = os.path.split(os.path.realpath(__file__))
repo_path, _ = os.path.split(repo_path)
sys.path.append(repo_path)
from utils.seed import seed_all
from utils.config import CFG
from utils.dataset import get_dataset
from utils.model import get_model
from utils.logger import get_logger
from utils.io_tools import dict_to, _create_directory
import utils.checkpoint as checkpoint
import datasets.io_data as SemanticKittiIO

def publish_coordinates(coordinates, publisher):
    coordinates = coordinates[:, [0, 2, 1]]
    coordinates_msg = Float64MultiArray()
    for coordinate in coordinates:
        # print(f"coordinate : {coordinate}")
        coordinates_msg.data.extend(coordinate)
    publisher.publish(coordinates_msg)

def test(model, dset, _cfg, logger, out_path_root, coordinates_publisher):
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    model = model.to(device=device)
    logger.info('=> Passing the network on the test set...')
    model.eval()
    inv_remap_lut = dset.dataset.get_inv_remap_lut()
    inference_time = []
   
    with torch.no_grad():
        for t, (data, indices) in enumerate(dset):
            data = dict_to(data, device)
            
            # Record the inference start time
            inference_start_time = time.time()

            scores = model(data)
            
            # Record the inference end time
            inference_end_time = time.time()
            
            # Log the inference time of each sample
            inference_time.append(inference_end_time - inference_start_time)

            for key in scores:
                scores[key] = torch.argmax(scores[key], dim=1).data.cpu().numpy()

            curr_index = 0
            for score in scores['pred_semantic_1_1']:
                input_filename = dset.dataset.filepaths['occupancy'][indices[curr_index]]
                print(input_filename)

                voxel_occupancy = SemanticKittiIO._read_occupancy_SemKITTI(input_filename)
                voxel_occupancy = voxel_occupancy.reshape(256, 32, 256)
                voxel_mask = voxel_occupancy.ravel() == 1
                voxel_occupied_count = np.count_nonzero(voxel_mask)
                score_mask = score.ravel() > 0
                score_occupied_count = np.count_nonzero(score_mask)
                intersection = np.logical_and(voxel_mask, score_mask)
                intersection_count = np.count_nonzero(intersection)
                non_intersection = np.logical_and(score_mask, np.logical_not(voxel_mask))
                non_intersection_coordinates = np.column_stack(np.nonzero(non_intersection.reshape(256, 32, 256)))

                publish_coordinates(non_intersection_coordinates, coordinates_publisher)
                
                # score = np.moveaxis(score, [0, 1, 2], [0, 2, 1]).reshape(-1).astype(np.uint16)
                # score = inv_remap_lut[score].astype(np.uint16)
                
                # filename, extension = os.path.splitext(os.path.basename(input_filename))
                # out_filename = os.path.join(out_path_root, 'predictions', filename + '.label')
                # _create_directory(os.path.dirname(out_filename))
                # score.tofile(out_filename)
             
                # os.remove(input_filename)
                # input_velodyne = input_filename.replace('voxels', 'velodyne')
                # os.remove(input_velodyne)
                # curr_index += 1
  
    return inference_time

def main():
    rospy.init_node("inference_node")
    coordinates_publisher = rospy.Publisher('/non_intersection_coordinates', Float64MultiArray, queue_size=1000)  
    torch.backends.cudnn.enabled = True
    seed_all(0)  
     
    weights_f = rospy.get_param('~weights_file')
    dataset_f = rospy.get_param('~dataset_root')
    out_path_root = rospy.get_param('~output_path')
    
    assert os.path.isfile(weights_f), '=> No file found at {}'
    checkpoint_path = torch.load(weights_f)
    config_dict = checkpoint_path.pop('config_dict')
    config_dict['DATASET']['ROOT_DIR'] = dataset_f
    _cfg = CFG()
    _cfg.from_dict(config_dict)
    _cfg.data = config_dict  # 添加这一行
    logger = get_logger(out_path_root, 'logs_test.log')
    logger.info('============ Test weights: "%s" ============\n' % weights_f)    
    while not rospy.is_shutdown():      
        logger.info('=> Loading network architecture...')
        dataset = get_dataset(_cfg)['test']
        model = get_model(_cfg, dataset.dataset)
        logger.info('=> Loading network weights...')
        model = checkpoint.load_model(model, weights_f, logger)
        rate = rospy.Rate(10)  
        inference_time = test(model, dataset, _cfg, logger, out_path_root, coordinates_publisher)  
        logger.info('=> ============ Network Test Done ============')
        logger.info('Inference time per frame is %.4f seconds\n' % (np.sum(inference_time) / 1.0))
        rate.sleep()

if __name__ == '__main__':
    main()
