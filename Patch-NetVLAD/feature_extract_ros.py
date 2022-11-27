#!/usr/bin/env python

'''

Modifier: Yilin Zhu             zhuylin25@mail2.sysu.edu.cn

Copyright (c) 2021 Stephen Hausler, Sourav Garg, Ming Xu, Michael Milford and Tobias Fischer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


Extracts Patch-NetVLAD local and NetVLAD global features from a given directory of images.

Configuration settings are stored in configs folder, with compute heavy performance or light-weight alternatives
available.

Features are saved into a nominated output directory, with one file per image per patch size.

Code is dynamic and can be configured with essentially *any* number of patch sizes, by editing the config files.
'''

import sys, os, time
# add parent folder to search path
# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))
# print(sys.path)

import argparse
import configparser
from os.path import join, exists, isfile
from os import makedirs

from tqdm.auto import tqdm
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
import numpy as np

# ros相关服务
import rospy
from netvlad_tf_test.srv import CompactImg, CompactImgRequest, CompactImgResponse

from patchnetvlad.tools.datasets import PlaceDataset
from patchnetvlad.models.models_generic import get_backend, get_model, get_pca_encoding
from patchnetvlad.tools import PATCHNETVLAD_ROOT_DIR


class FeatureExtract:
    def __init__(self) -> None:
        parser = argparse.ArgumentParser(description='Patch-NetVLAD-Feature-Extract')
        parser.add_argument('--config_path', type=str, default=join(PATCHNETVLAD_ROOT_DIR, 'configs/gacm.ini'),
                        help='File name (with extension) to an ini file that stores most of the configuration data for patch-netvlad')
        parser.add_argument('--dataset_file_path', type=str, default=join(PATCHNETVLAD_ROOT_DIR, 'dataset_imagenames/gacm_submapImg_index.txt'),
                        help='Full path (with extension) to a text file that stores the save location and name of all images in the dataset folder')
        parser.add_argument('--dataset_root_dir', type=str, default='',
                        help='If the files in dataset_file_path are relative, use dataset_root_dir as prefix.')
        # parser.add_argument('--output_features_dir', type=str, default=join(PATCHNETVLAD_ROOT_DIR, 'output_features'),
        #                 help='Path to store all patch-netvlad features')
        parser.add_argument('--nocuda', action='store_true', help='If true, use CPU only. Else use GPU.')

        self.opt = parser.parse_args()
    
    # 开始进行服务
    def main(self):

        configfile = self.opt.config_path
        assert os.path.isfile(configfile)
        config = configparser.ConfigParser()
        config.read(configfile)

        cuda = not self.opt.nocuda
        if cuda and not torch.cuda.is_available():
            raise Exception("No GPU found, please run with --nocuda")

        device = torch.device("cuda" if cuda else "cpu")

        encoder_dim, encoder = get_backend()

        # must resume to do extraction
        if config['global_params']['num_pcs'] != '0':
            resume_ckpt = config['global_params']['resumePath'] + config['global_params']['num_pcs'] + '.pth.tar'
        else:
            resume_ckpt = config['global_params']['resumePath'] + '.pth.tar'

        # backup: try whether resume_ckpt is relative to PATCHNETVLAD_ROOT_DIR
        if not isfile(resume_ckpt):
            resume_ckpt = join(PATCHNETVLAD_ROOT_DIR, resume_ckpt)
            if not isfile(resume_ckpt):
                from download_models import download_all_models
                download_all_models(ask_for_permission=True)

        if isfile(resume_ckpt):
            print("=> loading checkpoint '{}'".format(resume_ckpt))
            checkpoint = torch.load(resume_ckpt, map_location=lambda storage, loc: storage)
            if config['global_params']['num_pcs'] != '0':
                assert checkpoint['state_dict']['WPCA.0.bias'].shape[0] == int(config['global_params']['num_pcs'])
            config['global_params']['num_clusters'] = str(checkpoint['state_dict']['pool.centroids'].shape[0])

            if config['global_params']['num_pcs'] != '0':
                use_pca = True
            else:
                use_pca = False
            model = get_model(encoder, encoder_dim, config['global_params'], append_pca_layer=use_pca)
            model.load_state_dict(checkpoint['state_dict'])
            
            if int(config['global_params']['nGPU']) > 1 and torch.cuda.device_count() > 1:
                model.encoder = nn.DataParallel(model.encoder)
                # if opt.mode.lower() != 'cluster':
                model.pool = nn.DataParallel(model.pool)
        
            model = model.to(device)
            print("=> loaded checkpoint '{}'".format(resume_ckpt, ))
        else:
            raise FileNotFoundError("=> no checkpoint found at '{}'".format(resume_ckpt))

        self.config = config
        self.device = device
        self.model = model
        # feature_extract(dataset, model, device, self.opt, config)
        
        # ros设置
        rospy.init_node("feature_server_node")
        feature_server = rospy.Service("/compact_image", CompactImg, self.feature_server_callback)
        self.printGreen("Ready to convert Img to Features!")
        rospy.spin()

        torch.cuda.empty_cache()  # garbage clean GPU memory, a bug can occur when Pytorch doesn't automatically clear the
                                  # memory after runs
    
    # 得到全局描述子
    def feature_extract(self, eval_set):

        pool_size = int(self.config['global_params']['num_pcs'])

        test_data_loader = DataLoader(dataset=eval_set, num_workers=int(self.config['global_params']['threads']),
                                    batch_size=int(self.config['feature_extract']['cacheBatchSize']),
                                    shuffle=False, pin_memory=(not self.opt.nocuda))

        self.model.eval()
        with torch.no_grad():
            tqdm.write('====> Extracting Features')
            db_feat = np.empty((len(eval_set), pool_size), dtype=np.float32)

            for iteration, (input_data, indices) in \
                    enumerate(tqdm(test_data_loader, position=1, leave=False, desc='Test Iter'.rjust(15)), 1):
                indices_np = indices.detach().numpy()
                input_data = input_data.to(self.device)
                image_encoding = self.model.encoder(input_data)
                if self.config['global_params']['pooling'].lower() == 'patchnetvlad':
                    vlad_local, vlad_global = self.model.pool(image_encoding)

                    # 只保存全局描述子
                    vlad_global_pca = get_pca_encoding(self.model, vlad_global)
                    db_feat[indices_np, :] = vlad_global_pca.detach().cpu().numpy()
                    

                else:
                    vlad_global = self.model.pool(image_encoding)
                    vlad_global_pca = get_pca_encoding(self.model, vlad_global)
                    db_feat[indices_np, :] = vlad_global_pca.detach().cpu().numpy()

        return db_feat

    # 回调函数
    def feature_server_callback(self, req):

        print("\nreceive img dir: " + req.req_img_name)
        
        res = CompactImgResponse()
        if(not isfile(req.req_img_name)):
            self.pringRed("No img file in this path, return void features (all value -100)!")
            res.res_des = np.ones(int(self.config['global_params']['num_pcs']), dtype=np.float32) * -100
            return res

        start_t = time.perf_counter()
        with open(self.opt.dataset_file_path, 'w') as data_txt:
            data_txt.write(req.req_img_name)

        dataset = PlaceDataset(None, self.opt.dataset_file_path, self.opt.dataset_root_dir, None, self.config['feature_extract'])
        
        res.res_des = np.float32(self.feature_extract(dataset)[0])

        self.printGreen("\nFeatures (len: {}) succssfully create! working time: {:.3}s".format(int(self.config['global_params']['num_pcs']), time.perf_counter() - start_t))
        
        return res

    def printGreen(self, outstr):
        print("\033[1;32m%s\033[0m"%outstr)
    
    def pringRed(self, outstr):
        print("\033[1;31m%s\033[0m"%outstr)

if __name__ == "__main__":
    server = FeatureExtract()
    server.main()
