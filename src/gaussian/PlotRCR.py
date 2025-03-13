import numpy as np
import Config
import pickle
from copy import deepcopy
from scipy.stats import multivariate_normal
from src.data_structures.state import x_bins,y_bins,theta_bins,grab_bins, xbin_factor, ybin_factor, thetabin_factor,x_range,y_range
from math import radians, floor, isnan
import openravepy as orpy
from openravepy import *
from PIL import Image
from src.data_gen.DataGenerator import DataGenerator

class PlotRCR(DataGenerator):
    def __init__(self,env_list,visualize = False,num_samples=100):
        super(PlotRCR, self).__init__(env_name=env_list[0],number_of_configs=0,number_of_mp=0,visualize=visualize)
        self.env_name = env_list
        self.num_samples = num_samples
        self.visualize = visualize
        print(env_list)
        self.file_name = "segragated_mvg_param_"+"env0"+"_to_"+"env0"+"_data.p"
        self.data = self.load_data() 
        self.active_markers = []
        self.plot_dic = {}
        self.point_size = 0.015
        self.draw_style = 1
        self.dist_dic = {}
        self.image_arr, self.image_bins = self.load_raster_scan()
        self.pixel_factor_x = x_range/self.image_bins[0]
        self.pixel_factor_y = y_range/self.image_bins[1]

    def load_raster_scan(self):
        img_arr = np.load(Config.DATA_MISC_DIR+self.env_name[0]+"/"+self.env_name[0]+".npy")        
        size = img_arr.shape[:2]
        return img_arr, size
    
    def load_data(self):
        print("loading {} data".format("env0"))
        data_load = pickle.load(open(Config.DATA_MISC_DIR+self.file_name))
        
        return data_load
    
    def get_prob(self,bin_set,prim_obj_name,sec_obj_name,grip,rcr_num):
        bin_arr = np.array(bin_set)
        gaus = self.dist_dic[prim_obj_name][sec_obj_name][grip][rcr_num]["distribution"]

        return gaus.pdf(bin_arr)

    def check_dirac(self,arr):
        for a in range(arr.shape[0]):
            for b in range(arr.shape[1]):
                if not(isnan(arr[a,b])):
                    return False
        return True
        
    def gen_dist(self):
        for prim_obj_name in self.data.keys():
            prim_dist_dic = {}
            for sec_obj_name in self.data[prim_obj_name].keys():
                sec_dist_dic = {}
                for grip in self.data[prim_obj_name][sec_obj_name].keys():
                    sec_dist_dic[grip] = []
                    rcr_list = self.data[prim_obj_name][sec_obj_name][grip]
                    for rcr in rcr_list:
                        temp_dic = {}
                        mean = rcr["mean"]
                        cov_mat = rcr["cov_mat"]

                        
                        try:
                            gaus = multivariate_normal(mean = mean,cov= cov_mat)
                        except:
                            import IPython;IPython.embed()
                            if self.check_dirac(cov_mat):
                                gaus = "dirac_delta"
                                max_prob = 1.0
                            else:
                                self.get_singular_samples(self,cov_mat,mean)
                            samples = np.random.multivariate_normal(mean = mean, cov = cov_mat,size=1000)
                            probs = gaus.pdf(np.array(samples))
                            max_prob = max(probs)

                        temp_dic["distribution"] = gaus
                        temp_dic["max_prob"] = max_prob
                        sec_dist_dic[grip].append(temp_dic)
            
                prim_dist_dic[sec_obj_name] = sec_dist_dic
            
            self.dist_dic[prim_obj_name] = prim_dist_dic              

    def sample_distribution(self,prim_obj_name,num_samples, percent = 0.0):
        prim_obj_dic = self.data[prim_obj_name]
        sample_prim_obj_dic = {}
        print("sampling for {}".format(prim_obj_name))

        for sec_obj_name in prim_obj_dic.keys():
            sample_prim_obj_dic[sec_obj_name] = {}
            for grip in prim_obj_dic[sec_obj_name].keys():
                sample_prim_obj_dic[sec_obj_name][grip]  = []
                rcr_list = prim_obj_dic[sec_obj_name][grip]
                for rcr_num in range(len(rcr_list)):
                    sample_list = []                
                    mean = rcr_list[rcr_num]["mean"]
                    cov_mat = rcr_list[rcr_num]["cov_mat"]
                    i = 0

                    while i < num_samples:
                        if self.check_dirac(cov_mat):
                            sample = mean
                            sample_prob = 1.0
                            max_prob = 1.0

                        else:
                            sample = np.random.multivariate_normal(mean = mean, cov = cov_mat)
                            sample_prob = self.get_prob(sample,prim_obj_name= prim_obj_name, sec_obj_name= sec_obj_name,grip=grip,rcr_num=rcr_num)
                            max_prob = self.dist_dic[prim_obj_name][sec_obj_name][grip][rcr_num]["max_prob"]

                        if sample_prob >= (0.5-(percent*0.5))*max_prob and sample_prob <= (1-(0.5*percent))*max_prob:
                            sample = list(sample)
                            sample.append(sample_prob)
                            sample_list.append(sample)
                            i +=1
                    # else:
                    #     sample_list = np.random.multivariate_normal(mean = mean, cov = cov_mat,size=num_samples)
                    #     max_prob = self.dist_dic[prim_obj_name][sec_obj_name][grip][rcr_num]["max_prob"]
                    #     for sample in sample_list:
                    #         sample = list(sample)
                    #         sample_prob = self.get_prob(sample,prim_obj_name= prim_obj_name, sec_obj_name= sec_obj_name,grip=grip,rcr_num=rcr_num)
                    #         sample.append(sample_prob)
                   
                    sample_prim_obj_dic[sec_obj_name][grip].append(sample_list)
        # import IPython
        # IPython.embed()
        return sample_prim_obj_dic

    def bins2configSpace(self,bin_index):
        count_arr = [x_bins,y_bins,theta_bins,grab_bins]
        config_arr = []
        bin_factor = xbin_factor
        for i in range(len(bin_index)-1):
            index = bin_index[i]
            bin_count = count_arr[i]
            if i < 2:
                val = ((index-bin_count/2)+0.5)*bin_factor
            elif i == 2:
                val = ((index-bin_count/2)+0.5)*thetabin_factor
            else:
                val = index
            
            config_arr.append(val)
        
        return config_arr
    
    def transform_from_xyt(self,xyt):
        theta = xyt[2]
        angleAxis = np.array((0,0,theta))
        rot = orpy.rotationMatrixFromAxisAngle(angleAxis)
        transform = np.eye(4)
        transform[:3,:3] = rot
        transform[:2,3] = xyt[:2]

        return transform
    
    def xyt_from_transform(self,transform):
        x = transform[0,3]
        y = transform[1,3]
        rot = transform[:3,:3]
        theta = orpy.axisAngleFromRotationMatrix(rot)[2]

        return [x,y,theta]
    
    def relative_to_world(self,transform,prim_obj_name):
        if prim_obj_name != 'gripper' and prim_obj_name != 'floorwalls':
            obj = self.env.GetKinBody(prim_obj_name)
            obj_tf = obj.GetTransform()
        else:
            obj_tf = self.robot.GetLink('base').GetTransform()

        cr_in_wf = obj_tf.dot(transform)
        return cr_in_wf
    
    def cs_to_pixel(self,cs):
        [x,y,theta] = cs
        x_px = floor(x/self.pixel_factor_x)
        y_px = floor(y/self.pixel_factor_y)

        return [x_px,y_px,theta]
    
    def get_plot_data(self,percent = 0.85):
        self.gen_dist()
        for prim_obj_name in self.data.keys():
            prim_samples = self.sample_distribution(prim_obj_name,self.num_samples,percent)
            prim_points = {}
            # import IPython
            # IPython.embed()
            for sec_obj_name in prim_samples.keys():
                sec_obj_dic  = prim_samples[sec_obj_name]
                sec_rcr = []
                sec_points = []

                for grip in sec_obj_dic.keys():
                    sec_rcr.extend(sec_obj_dic[grip])

                for rcr_points in sec_rcr:    
                    pixels_rcr = []

                    for i in range(self.num_samples):
                        sample = rcr_points[i]
                        cs = self.bins2configSpace(sample)
                        trans = self.transform_from_xyt(cs)
                        world_trans = self.relative_to_world(trans,prim_obj_name)
                        world_cs = self.xyt_from_transform(world_trans)
                        world_cs = self.cs_to_pixel(world_cs)
                        world_cs.append(round(sample[-2])) #the grabbed bins
                        world_cs.append(sample[-1]) #probability value
                        pixels_rcr.append(world_cs)
                    
                    sec_points.append(pixels_rcr)

                prim_points[sec_obj_name] = sec_points
            
            self.plot_dic[prim_obj_name] = prim_points
        
        # return self.plot_dic
    
    def randomize_env(self):
        if self.active_markers is not None:
            for marker in self.active_markers:
                marker.Close()

            self.active_markers = []

        super(PlotRCR,self).randomize_env()
        self.get_plot_data()
    
    def plot_data_points(self,obj_name):
        plot_points = []
        if self.active_markers is not None:
            for marker in self.active_markers:
                marker.Close()
            self.active_markers = []
            
        if not self.visualize:
            self.visualize = True
            self.env.SetViewer('qtcoin')

        plot_rcr = []
        for prim_obj_name in self.plot_dic.keys():
            if prim_obj_name != obj_name:
                plot_rcr.extend(self.plot_dic[prim_obj_name][obj_name])

        for prim_obj_name in self.plot_dic.keys():
            if prim_obj_name != obj_name:
                ps = self.plot_dic[prim_obj_name][obj_name]
                max_prob = self.dist_dic[prim_obj_name][obj_name]["max_prob"]
                for point in ps:
                    prob = point[-1]
                    point[-1] = prob/max_prob
                    plot_points.append(point)

        for point in plot_points:
            p = point[:2]
            # prob_factor = 1/(point[-1]*100)
            prob_factor = point[-1]*2
            if int(point[-2]):
                p.append(0.07)
                color = [0,0,prob_factor]
            else:
                p.append(0.15)
                color = [prob_factor,0,0]           

            self.active_markers.append(self.env.plot3(np.array(p),pointsize=self.point_size*prob_factor,drawstyle=self.draw_style,colors=np.array(color)))
        
        return 1
    
    def map_image(self,obj_name):
        image = self.image_arr[:,:,:3]

        plot_rcr = []

        for prim_obj_name in self.plot_dic.keys():
            if prim_obj_name != obj_name:
                plot_rcr.extend(self.plot_dic[prim_obj_name][obj_name])
            
        for rcr in plot_rcr:
            r = np.random.uniform(255)/255
            g = np.random.uniform(255)/255
            b = np.random.uniform(255)/255
            # color = [r,g,b]
            for point in rcr:
                [x,y] = point[:2]
                # import IPython;IPython.embed()
                x = int(x)
                y = int(y)
                image[x,y,0] = r
                image[x,y,1] = g
                image[x,y,2] = b
                        
        self.image = Image.fromarray(image, 'RGB')
        self.image.show()

    def save_image(self,obj_name):
        self.img.save('{}_w.r.t._others.png'.format(obj_name))