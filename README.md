<div align="center">   
  
# 🤖 HE-Nav: A High-Performance and Efficient Navigation System for Aerial-Ground Robots in Occluded Environments
*We will open source the complete code after the paper is accepted*
</div>

## 📢 News
- [2024/07]: The training logs of LBSCNet and the navigation experiment logs of HE-Nav can be downloaded from this [link](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/u3009632_connect_hku_hk/EspXg1lHAElIpMbWldFNpxgB5eHx80JQ4sdtIn0-ymcZ5w?e=NQ8JDK).
- [2024/04]: The [3D model ](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/u3009632_connect_hku_hk/ERX7ejbV3xdOkLQe5SMgGG0Bh6D1qGd-9vg5iMWpi8VQsw?e=H07haj) in the simulation environment can be downloaded in OneDrive.
- [2024/04]: 🔥 We released the code of HE-Nav in the simulation environment. The pre-trained model can be downloaded at  [OneDrive](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/u3009632_connect_hku_hk/Ef07DNxypO1KhwuMNbthmP8BgIeDdXJyaeq4uwi6hFKgRw?e=y2LYv2)

</br>

## 🛠️ Installation
The code was tested with `python=3.6.9`, as well as `pytorch=1.10.0+cu111` and `torchvision=0.11.2+cu111`. 

Please follow the instructions [here](https://pytorch.org/get-started/locally/) to install both PyTorch and TorchVision dependencies. Installing both PyTorch and TorchVision with CUDA support is strongly recommended.

1. Clone the repository locally:

```
 git clone https://github.com/jmwang0117/HE-Nav.git
```
2. We recommend using **Docker** to run the project, which can reduce the burden of configuring the environment, you can find the Dockerfile in our project, and then execute the following command:
```
 docker build . -t skywalker_robot -f Dockerfile
```
3. After the compilation is complete, use our **one-click startup script** in the same directory:
```
 bash create_container.sh
```

 **Pay attention to switch docker image**

4. Next enter the container and use git clone our project
```
 docker exec -it robot bash
```
5. Then catkin_make compiles this project
```
 apt update && sudo apt-get install libarmadillo-dev ros-melodic-nlopt

```
6. Run the following commands 
```
pip install pyyaml
pip install rospkg
pip install imageio
catkin_make
source devel/setup.bash
sh src/run_sim.sh
```

You've begun this project successfully; **enjoy yourself!**


## 💽 Dataset

- [x] SemanticKITTI


## 🤗 AGR-Family Works

* [HE-Nav](https://jmwang0117.github.io/HE-Nav/) (Submitted to RA-L'24): The First AGR Navigation System that achieves ESDF-free Path Planning in Heavily Cluttered Environments.
* [AGRNav](https://github.com/jmwang0117/AGRNav) (ICRA'24): The First AGR Navigation System with Occlusion Awareness.


## 🏆Acknowledgement

Many thanks to these excellent open source projects:
- [AGRNav](https://github.com/jmwang0117/AGRNav)
- [Prometheus](https://github.com/amov-lab/Prometheus)
- [LMSCNet](https://github.com/astra-vision/LMSCNet)
- [semantic-kitti-api](https://github.com/PRBonn/semantic-kitti-api)
- [Terrestrial-Aerial-Navigation](https://github.com/ZJU-FAST-Lab/Terrestrial-Aerial-Navigation)
- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)

